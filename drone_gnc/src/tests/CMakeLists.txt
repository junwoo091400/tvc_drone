cmake_minimum_required(VERSION 3.0.2)
project(tests)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(../control)
include_directories(../guidance)
include_directories(../models)

add_executable(control_mpc_test control_mpc_test.cpp)
set_target_properties(control_mpc_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(control_mpc_test drone_mpc Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(guidance_test guidance_test.cpp)
set_target_properties(guidance_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(guidance_test drone_guidance_lib Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(terminal_cost_test terminal_cost_test.cpp)
set_target_properties(terminal_cost_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(terminal_cost_test mpc_utils drone_model Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(full_trajectory_tracking_test full_trajectory_tracking_test.cpp)
set_target_properties(full_trajectory_tracking_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(full_trajectory_tracking_test drone_mpc drone_guidance_lib Eigen3::Eigen OsqpEigen::OsqpEigen)