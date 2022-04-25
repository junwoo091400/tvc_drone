cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(../models)

add_library(mpc_utils mpc_utils.cpp)
target_link_libraries(mpc_utils Eigen3::Eigen)

add_library(drone_mpc drone_mpc.cpp)
target_link_libraries(drone_mpc mpc_utils Eigen3::Eigen OsqpEigen::OsqpEigen)