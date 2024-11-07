cmake_minimum_required(VERSION 2.8.3)
project(state_predictor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

cs_add_library(${PROJECT_NAME} src/state_predictor.cpp 
    src/quad_ext_state_estimate.cpp)

cs_install()
cs_export()
