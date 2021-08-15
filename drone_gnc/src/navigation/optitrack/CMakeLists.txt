cmake_minimum_required(VERSION 3.0.2)
project(navigation_optitrack)

find_package(Eigen3 REQUIRED)

add_executable(drone_navigation_optitrack ../drone_EKF.hpp drone_navigation_node_optitrack.cpp drone_EKF_optitrack.hpp)
target_link_libraries(drone_navigation_optitrack Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(drone_navigation_optitrack drone_gnc_generate_messages_cpp)
