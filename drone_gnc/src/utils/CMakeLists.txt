cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(Eigen3 3.3.20 REQUIRED)

add_executable(drone_fsm drone_fsm_node.cpp)
target_link_libraries(drone_fsm ${catkin_LIBRARIES})
add_dependencies(drone_fsm drone_gnc_generate_messages_cpp)


add_executable(control_devel control_devel.cpp)
target_link_libraries(control_devel Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(control_devel drone_gnc_generate_messages_cpp)