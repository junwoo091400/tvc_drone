cmake_minimum_required(VERSION 3.0.2)
project(utils)

add_executable(drone_fsm drone_fsm_node.cpp)
target_link_libraries(drone_fsm ${catkin_LIBRARIES})
add_dependencies(drone_fsm drone_gnc_generate_messages_cpp)