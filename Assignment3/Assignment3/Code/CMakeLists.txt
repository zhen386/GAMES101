cmake_minimum_required(VERSION 3.10)
project(Rasterizer)


FIND_PACKAGE(OpenCV REQUIRED)

# TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/Eigen)
# include_directories(${EIGEN3_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)



add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer Eigen3::Eigen ${OpenCV_LIBRARIES})