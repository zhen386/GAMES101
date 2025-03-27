cmake_minimum_required(VERSION 3.30)
project(GAMES101)

set(CMAKE_CXX_STANDARD 14)

include_directories(Assignment1)
include_directories(Assignment2)

add_executable(GAMES101
    Assignment1/main.cpp
    Assignment1/rasterizer.cpp
    Assignment1/rasterizer.hpp
    Assignment1/Triangle.cpp
    Assignment1/Triangle.hpp
    Assignment2/global.hpp
    Assignment2/main.cpp
    Assignment2/rasterizer.cpp
    Assignment2/rasterizer.hpp
    Assignment2/Triangle.cpp
    Assignment2/Triangle.hpp)
