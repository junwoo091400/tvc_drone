cmake_minimum_required(VERSION 3.0.2)
project(navigation_pixhawk)

find_package(Eigen3 REQUIRED)

add_executable(drone_navigation_pixhawk ../drone_EKF.hpp drone_navigation_node_pixhawk.cpp drone_EKF_pixhawk.hpp)
target_link_libraries(drone_navigation_pixhawk Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(drone_navigation_pixhawk drone_gnc_generate_messages_cpp)