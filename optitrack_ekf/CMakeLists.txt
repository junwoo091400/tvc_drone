cmake_minimum_required(VERSION 2.8.3)
project(optitrack_ekf)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/../../build)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf tf2_ros rocket_utils)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vrpn_client_ros
  CATKIN_DEPENDS geometry_msgs tf2_ros rocket_utils
  DEPENDS VRPN
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${VRPN_INCLUDE_DIR}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_subdirectory(src)