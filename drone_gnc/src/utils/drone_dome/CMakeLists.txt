cmake_minimum_required(VERSION 3.0.2)
project(drone_dome)

add_executable(optitrack_client optitrack_client.cpp)
target_link_libraries(optitrack_client ${catkin_LIBRARIES})
