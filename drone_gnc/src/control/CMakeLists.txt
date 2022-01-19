cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(../models)

add_library(mpc_utils mpc_utils.cpp)
set_target_properties(mpc_utils PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(mpc_utils drone_model Eigen3::Eigen)

add_library(drone_mpc drone_mpc.cpp)
set_target_properties(drone_mpc PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(drone_mpc drone_model mpc_utils Eigen3::Eigen OsqpEigen::OsqpEigen)