cmake_minimum_required(VERSION 3.0.2)
project(guidance)

include_directories(../models)

add_library(drone_guidance_lib drone_guidance.cpp)
target_link_libraries(drone_guidance_lib Eigen3::Eigen)