// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "kiss_icp_cpp/core/Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

#include "kiss_icp_cpp/core/VoxelHashMap.hpp"
#include "kiss_icp_cpp/core/VoxelUtils.hpp"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace {
inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, closest_neighbor);
                }
            });
        });
    return correspondences;
}

LinearSystem BuildLinearSystem(const Correspondences &correspondences, const double kernel_scale) {
    auto compute_jacobian_and_residual = [](const auto &correspondence) {
        const auto &[source, target] = correspondence;
        const Eigen::Vector3d residual = source - target;
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
        return std::make_tuple(J_r, residual);
    };

    auto sum_linear_systems = [](LinearSystem a, const LinearSystem &b) {
        a.first += b.first;
        a.second += b.second;
        return a;
    };

    auto GM_weight = [&](const double &residual2) {
        return square(kernel_scale) / square(kernel_scale + residual2);
    };

    using correspondence_iterator = Correspondences::const_iterator;
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(correspondence);
                    const double w = GM_weight(residual.squaredNorm());
                    return LinearSystem(J_r.transpose() * w * J_r,        // JTJ
                                        J_r.transpose() * w * residual);  // JTr
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);

    return {JTJ, JTr};
}
}  // namespace

namespace kiss_icp {

Registration::Registration(int max_num_iteration, double convergence_criterion, int max_num_threads)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      // Only manipulate the number of threads if the user specifies something greater than 0
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()) {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

Sophus::SE3d Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d> &frame,
                                            const VoxelHashMap &voxel_map,
                                            const Sophus::SE3d &initial_guess,
                                            const double max_distance,
                                            const double kernel_scale,
                                            double &iteration_number) {
    if (voxel_map.Empty()) return initial_guess;

    // Equation (9)
    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        // Equation (10)
        const auto correspondences = DataAssociation(source, voxel_map, max_distance);
        // Equation (11)
        const auto &[JTJ, JTr] = BuildLinearSystem(correspondences, kernel_scale);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        // Equation (12)
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        iteration_number = j;
        if (dx.norm() < convergence_criterion_) break;
    }
    // Spit the final transformation
    return T_icp * initial_guess;
}

namespace {

bool EstimatePlane(const std::vector<Eigen::Vector3d> &neighbors, Eigen::Vector3d &normal) {
    constexpr int kMinNeighbors = 5;
    if (static_cast<int>(neighbors.size()) < kMinNeighbors) {
        return false;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &point : neighbors) {
        mean += point;
    }
    mean /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : neighbors) {
        const Eigen::Vector3d delta = point - mean;
        covariance += delta * delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    const double lambda_min = solver.eigenvalues()(0);
    const double lambda_max = solver.eigenvalues()(2);
    if (lambda_max <= 1e-8 || lambda_min / lambda_max > 0.25) {
        return false;
    }
    normal = solver.eigenvectors().col(0);
    if (normal.norm() < 1e-8) {
        return false;
    }
    normal.normalize();
    return true;
}

struct PlaneAccumulation {
    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();
    int correspondence_count = 0;
};

void AccumulatePlanePoint(PlaneAccumulation &accumulation,
                          const Eigen::Vector3d &point_body,
                          const Sophus::SE3d &pose,
                          const kiss_icp::VoxelHashMap &voxel_map,
                          const double max_distance,
                          const double kernel_scale,
                          std::vector<Eigen::Vector3d> &neighbors) {
    constexpr int kNeighborCount = 20;
    const Eigen::Vector3d point_world = pose * point_body;
    voxel_map.CollectNeighbors(point_world, kNeighborCount, neighbors);
    if (neighbors.empty()) {
        return;
    }
    const auto closest = std::min_element(
        neighbors.cbegin(), neighbors.cend(),
        [&](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
            return (lhs - point_world).squaredNorm() < (rhs - point_world).squaredNorm();
        });
    if ((*closest - point_world).norm() >= max_distance) {
        return;
    }

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    if (!EstimatePlane(neighbors, normal)) {
        return;
    }
    ++accumulation.correspondence_count;
    const Eigen::Vector3d normal_body = pose.so3().inverse() * normal;
    const double residual = normal.dot(point_world - *closest);
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.block<1, 3>(0, 0) = normal_body.transpose();
    jacobian.block<1, 3>(0, 3) = point_body.cross(normal_body).transpose();
    const double kernel2 = square(kernel_scale);
    const double weight = kernel2 / square(kernel_scale + residual * residual);
    accumulation.JTJ += jacobian.transpose() * weight * jacobian;
    accumulation.JTr += jacobian.transpose() * weight * residual;
}

bool SolveBodyIncrement(const PlaneAccumulation &accumulation, Eigen::Vector6d &dx) {
    constexpr double kDegeneracyRatio = 1e-3;
    constexpr double kDamping = 1e-6;
    const Eigen::Matrix6d &hessian = accumulation.JTJ;
    Eigen::Vector6d scale = hessian.diagonal().cwiseAbs().cwiseSqrt();
    for (int axis = 0; axis < 6; ++axis) {
        if (scale(axis) < 1e-9) {
            scale(axis) = 1.0;
        }
    }

    Eigen::Matrix6d hessian_normalized = hessian;
    Eigen::Vector6d gradient = accumulation.JTr;
    for (int row = 0; row < 6; ++row) {
        gradient(row) /= scale(row);
        for (int column = 0; column < 6; ++column) {
            hessian_normalized(row, column) /= scale(row) * scale(column);
        }
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix6d> eigen_solver(hessian_normalized);
    if (eigen_solver.info() != Eigen::Success) {
        return false;
    }
    const double lambda_max = eigen_solver.eigenvalues()(5);
    Eigen::Vector6d y = Eigen::Vector6d::Zero();
    if (lambda_max > 0.0) {
        for (int axis = 0; axis < 6; ++axis) {
            const double lambda = eigen_solver.eigenvalues()(axis);
            if (lambda < kDegeneracyRatio * lambda_max) {
                continue;
            }
            const Eigen::Vector6d direction = eigen_solver.eigenvectors().col(axis);
            y -= direction * (direction.dot(gradient) / (lambda + kDamping));
        }
    }
    dx = y.cwiseQuotient(scale);
    return dx.allFinite();
}

PlaneAccumulation BuildPlaneLinearSystem(const std::vector<Eigen::Vector3d> &frame,
                                         const Sophus::SE3d &pose,
                                         const kiss_icp::VoxelHashMap &voxel_map,
                                         const double max_distance,
                                         const double kernel_scale) {
    auto join = [](PlaneAccumulation left, const PlaneAccumulation &right) {
        left.JTJ += right.JTJ;
        left.JTr += right.JTr;
        left.correspondence_count += right.correspondence_count;
        return left;
    };
    return tbb::parallel_reduce(
        tbb::blocked_range<std::size_t>{0, frame.size()}, PlaneAccumulation{},
        [&](const tbb::blocked_range<std::size_t> &range, PlaneAccumulation accumulation) {
            std::vector<Eigen::Vector3d> neighbors;
            neighbors.reserve(128);
            for (std::size_t index = range.begin(); index < range.end(); ++index) {
                AccumulatePlanePoint(accumulation, frame[index], pose, voxel_map, max_distance,
                                     kernel_scale, neighbors);
            }
            return accumulation;
        },
        join);
}

}  // namespace

PlaneAlignResult Registration::AlignPointsToPlane(const std::vector<Eigen::Vector3d> &frame,
                                                  const VoxelHashMap &voxel_map,
                                                  const Sophus::SE3d &initial_guess,
                                                  const double max_distance,
                                                  const double kernel_scale,
                                                  double &iteration_number) {
    PlaneAlignResult result;
    result.pose = initial_guess;
    result.correction = Sophus::SE3d();
    iteration_number = 0.0;
    if (voxel_map.Empty() || frame.empty() || kernel_scale <= 0.0) {
        return result;
    }

    // Right-multiply the increment so rotation is about the vehicle, not the map origin.
    Sophus::SE3d pose = initial_guess;
    constexpr int kMinCorrespondences = 10;
    constexpr double kMaxStepTranslation = 0.5;
    constexpr double kMaxStepRotation = 0.2;
    for (int iteration = 0; iteration < max_num_iterations_; ++iteration) {
        const PlaneAccumulation accumulation =
            BuildPlaneLinearSystem(frame, pose, voxel_map, max_distance, kernel_scale);
        if (accumulation.correspondence_count < kMinCorrespondences) {
            break;
        }
        Eigen::Vector6d dx = Eigen::Vector6d::Zero();
        if (!SolveBodyIncrement(accumulation, dx)) {
            break;
        }
        if (dx.head<3>().norm() > kMaxStepTranslation) {
            dx.head<3>() *= kMaxStepTranslation / dx.head<3>().norm();
        }
        if (dx.tail<3>().norm() > kMaxStepRotation) {
            dx.tail<3>() *= kMaxStepRotation / dx.tail<3>().norm();
        }
        pose = pose * Sophus::SE3d::exp(dx);
        iteration_number = static_cast<double>(iteration + 1);
        if (dx.norm() < convergence_criterion_) {
            break;
        }
    }
    result.pose = pose;
    result.correction = pose * initial_guess.inverse();
    return result;
}

}  // namespace kiss_icp
