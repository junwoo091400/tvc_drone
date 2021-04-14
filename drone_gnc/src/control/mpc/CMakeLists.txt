cmake_minimum_required(VERSION 3.0.2)
project(mpc)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(drone_mpc drone_mpc.cpp drone_mpc.h)
target_link_libraries(drone_mpc Eigen3::Eigen OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(drone_mpc drone_gnc_generate_messages_cpp)