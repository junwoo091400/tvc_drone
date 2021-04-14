cmake_minimum_required(VERSION 3.0.2)
project(navigation)

add_executable(drone_navigation drone_navigation_node.cpp drone_EKF.cpp drone_EKF.h)
target_link_libraries(drone_navigation ${catkin_LIBRARIES})
add_dependencies(drone_navigation real_time_simulator_generate_messages_cpp )