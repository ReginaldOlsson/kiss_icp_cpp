# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.10)
project(kiss_icp_cpp VERSION 1.2.3 LANGUAGES CXX)

find_package(ament_cmake)
include_directories(${CMAKE_SOURCE_DIR}/include)
# Setup build options
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" ON)
option(USE_SYSTEM_TSL-ROBIN-MAP "Use system pre-installed tsl_robin" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
option(KISS_ICP_ENABLE_SIMD "Compile with native SIMD and 32-byte Eigen alignment" ON)

if(KISS_ICP_ENABLE_SIMD)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|aarch64|arm64)$")
    set(KISS_ICP_SIMD_FLAGS -march=native -mtune=native)
  else()
    set(KISS_ICP_SIMD_FLAGS)
    message(WARNING "KISS_ICP_ENABLE_SIMD is ON but no SIMD flags are known for ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
  # Force 32-byte Eigen ABI so AVX consumers (e.g. back_odom) can share Vector3d layouts.
  set(KISS_ICP_SIMD_DEFINES EIGEN_MAX_ALIGN_BYTES=32 EIGEN_MAX_STATIC_ALIGN_BYTES=32)
  if(KISS_ICP_SIMD_FLAGS)
    message(STATUS "kiss_icp_cpp SIMD enabled: ${KISS_ICP_SIMD_FLAGS}; Eigen align=32")
  endif()
endif()

# # ccache setup
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)
  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

# Set build type (repeat here for C++ only consumers)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(3rdparty/find_dependencies.cmake)
include(cmake/CompilerOptions.cmake)

add_library(kiss_icp_cpp SHARED src/core/Registration.cpp src/core/VoxelHashMap.cpp src/core/VoxelUtils.cpp
                                src/core/Preprocessing.cpp src/core/Threshold.cpp src/pipeline/KissICP.cpp)
# Proper public include interface
target_include_directories(
  kiss_icp_cpp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/kiss_icp_cpp>
                      $<INSTALL_INTERFACE:include> $<INSTALL_INTERFACE:include/kiss_icp_cpp>)
target_link_libraries(kiss_icp_cpp PRIVATE Eigen3::Eigen tsl::robin_map TBB::tbb Sophus::Sophus)
if(KISS_ICP_ENABLE_SIMD)
  if(KISS_ICP_SIMD_FLAGS)
    target_compile_options(kiss_icp_cpp PRIVATE ${KISS_ICP_SIMD_FLAGS})
  endif()
  # PUBLIC so dependents compile against the same Eigen ABI.
  target_compile_definitions(kiss_icp_cpp PUBLIC ${KISS_ICP_SIMD_DEFINES})
endif()

add_subdirectory(src/metrics)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library and export targets
install(
  TARGETS kiss_icp_cpp
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})

ament_package()
