cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(EIGEN_INCLUDE_DIR /usr/local/include/eigen3)

project(ransac LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)

include_directories(~/documents/pico_sdk/Include)
link_directories(~/documents/pico_sdk/Lib/x64)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(RANSAC
        src/main/random_engine.hpp

        src/main/models/point_t.hpp
        src/main/models/model_t.hpp
        src/main/models/line_t.hpp
        src/main/models/plane_t.hpp)

set(UTILITIES
        src/main/utilities/calculate.h

        src/main/utilities/stop_watch.hh
        src/main/utilities/stop_watch.cc
        src/main/utilities/range.hpp

        src/main/utilities/pico_sense.h)

add_library(pcl_ransac_cpu
        src/main/utilities/calculate.h
        src/main/utilities/calculate.cpp)

add_library(pcl_ransac_cuda
        src/main/utilities/stop_watch.hh
        src/main/utilities/stop_watch.cc
        src/main/utilities/calculate.h
        src/main/utilities/calculate.cu)
target_compile_features(pcl_ransac_cuda PUBLIC cxx_std_14)
set_target_properties(pcl_ransac_cuda PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_executable(ransac
        src/samples/cuda_test.cpp
        src/main/ransac.h

        ${UTILITIES}
        ${RANSAC})
target_link_libraries(ransac pcl_ransac_cuda)
target_link_libraries(ransac picozense_api)
target_link_libraries(ransac ${PCL_LIBRARIES})
