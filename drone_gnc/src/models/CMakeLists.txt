cmake_minimum_required(VERSION 3.0.2)
project(models)

find_package(Eigen3 REQUIRED)

add_library(drone_model drone_model.hpp)
set_target_properties(drone_model PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(drone_model Eigen3::Eigen)