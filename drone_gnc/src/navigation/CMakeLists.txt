cmake_minimum_required(VERSION 3.0.2)
project(navigation)

add_subdirectory(optitrack)
add_subdirectory(pixhawk)

find_package(catkin REQUIRED rosbag)
add_executable(kalman_tuner kalman_tuner.cpp optitrack/drone_EKF_optitrack.hpp)
target_link_libraries(kalman_tuner Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(kalman_tuner drone_gnc_generate_messages_cpp)
