cmake_minimum_required(VERSION 3.0.2)
project(nodes)


find_package(OsqpEigen REQUIRED)

add_executable(drone_control drone_control_node.cpp)
target_link_libraries(drone_control drone_mpc OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(drone_control drone_gnc_generate_messages_cpp)

add_subdirectory(mpc)