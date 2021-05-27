cmake_minimum_required(VERSION 3.0.2)
project(guidance)

find_package(OsqpEigen REQUIRED)

find_package(Eigen3 REQUIRED)

add_subdirectory(guidance_mpc)

add_executable(drone_guidance drone_guidance_node.cpp)
target_link_libraries(drone_guidance drone_guidance_mpc OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(drone_guidance drone_gnc_generate_messages_cpp)

add_executable(fixed_guidance fixed_guidance_node.cpp)
target_link_libraries(fixed_guidance Eigen3::Eigen ${catkin_LIBRARIES})
add_dependencies(fixed_guidance drone_gnc_generate_messages_cpp)