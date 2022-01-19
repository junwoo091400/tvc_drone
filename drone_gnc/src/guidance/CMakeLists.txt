cmake_minimum_required(VERSION 3.0.2)
project(guidance)

include_directories(../models)

add_library(drone_guidance_lib drone_guidance.cpp)
set_target_properties(drone_guidance_lib PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(drone_guidance_lib drone_model Eigen3::Eigen)