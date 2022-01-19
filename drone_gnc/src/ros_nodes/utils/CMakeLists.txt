cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(Eigen3 3.3.20 REQUIRED)

include_directories(../models)
include_directories(../control)
include_directories(../guidance)

add_executable(drone_fsm drone_fsm_node.cpp)
target_link_libraries(drone_fsm ${catkin_LIBRARIES})
add_dependencies(drone_fsm drone_gnc_generate_messages_cpp)

add_executable(drone_mavros_interface drone_mavros_interface.cpp)
target_link_libraries(drone_mavros_interface Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(drone_mavros_interface drone_gnc_generate_messages_cpp)