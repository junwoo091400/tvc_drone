cmake_minimum_required(VERSION 3.0.2)
project(drone_dome)

find_package(VRPN REQUIRED)

add_executable(optitrack_client optitrack_client.cpp simple_vrpn_client.cpp simple_vrpn_client.h)
target_link_libraries(optitrack_client ${VRPN_LIBRARIES} ${catkin_LIBRARIES})
