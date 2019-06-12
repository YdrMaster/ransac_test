cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ransac LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(HEADERS
        src/main/random_engine.hpp

        src/main/models/point_t.hpp
        src/main/models/model_t.hpp
        src/main/models/line_t.hpp
        src/main/models/plane_t.hpp)

include_directories(/home/user/Documents/pico_sdk/Include)
link_directories(/home/user/Documents/pico_sdk/Lib)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ransac src/samples/display.cpp src/main/ransac.h src/samples/range.hpp ${HEADERS})
target_link_libraries(ransac picozense_api)
target_link_libraries(ransac ${PCL_LIBRARIES})
