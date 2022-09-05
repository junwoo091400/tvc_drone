cmake_minimum_required(VERSION 3.0.2)

add_library(mpc_utils mpc_utils.cpp)
target_link_libraries(mpc_utils Eigen3::Eigen)

add_library(drone_mpc drone_mpc.cpp)
target_link_libraries(drone_mpc mpc_utils Eigen3::Eigen OsqpEigen::OsqpEigen)

add_library(drone_guidance drone_guidance.cpp)
target_link_libraries(drone_guidance Eigen3::Eigen)

if (BUILD_ROS)
  add_executable(drone_control_node drone_control_node.cpp)
  target_link_libraries(drone_control_node drone_mpc OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
  add_dependencies(drone_control_node drone_optimal_control_generate_messages_cpp)

  add_executable(drone_guidance_node drone_guidance_node.cpp)
  target_link_libraries(drone_guidance_node drone_guidance OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
  add_dependencies(drone_guidance_node drone_optimal_control_generate_messages_cpp)

  add_executable(fixed_guidance_node fixed_guidance_node.cpp)
  target_link_libraries(fixed_guidance_node Eigen3::Eigen ${catkin_LIBRARIES})
  add_dependencies(fixed_guidance_node drone_optimal_control_generate_messages_cpp)
endif ()