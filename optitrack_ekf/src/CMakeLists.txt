cmake_minimum_required(VERSION 3.0.2)
project(optitrack_ekf)

find_package(VRPN REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(optitrack_client optitrack_client.cpp vrpn/simple_vrpn_client.cpp vrpn/simple_vrpn_client.h)
target_link_libraries(optitrack_client ${VRPN_LIBRARIES} ${catkin_LIBRARIES})

add_executable(optitrack_ekf ekf_node.cpp extended_kalman_filter.hpp)
target_link_libraries(optitrack_ekf Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(optitrack_ekf ${optitrack_ekf_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})