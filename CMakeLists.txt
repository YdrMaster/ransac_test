cmake_minimum_required(VERSION 3.14)
project(ransac_test)

set(CMAKE_CXX_STANDARD 17)

add_executable(ransac_test main.cpp model.hpp random_engine.hpp ransac.h)
