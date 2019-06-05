cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ransac LANGUAGES CXX)

set(HEADERS
        src/main/ransac.h
        src/main/random_engine.hpp

        src/main/models/point_t.hpp
        src/main/models/model_t.hpp
        src/main/models/line_t.hpp
        src/main/models/plane_t.hpp)

link_directories(extern-lib/PicoZenseSDK_Windows_20190316_V2.3.9.2_DCAM710/Lib/x86)

add_executable(ransac src/samples/pico.cpp ${HEADERS})
target_link_libraries(ransac picozense_api)
