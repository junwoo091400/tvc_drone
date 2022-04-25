cmake_minimum_required(VERSION 3.0.2)
project(simulator_interface)

find_package(Eigen3 REQUIRED)

#add_executable(simulator_interface simulator_interface.cpp)
#target_link_libraries(simulator_interface Eigen3::Eigen ${catkin_LIBRARIES})
#add_dependencies(simulator_interface drone_optimal_control_generate_messages_cpp ${drone_optimal_control_EXPORTED_TARGETS})