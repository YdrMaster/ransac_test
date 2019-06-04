cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(ransac LANGUAGES CXX)

set(HEADERS
        src/main/ransac.h
        src/main/random_engine.hpp

        src/main/models/model_t.hpp
        src/main/models/line_t.hpp)

add_executable(ransac src/main.cpp ${HEADERS})
