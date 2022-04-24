cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(Eigen3 3.3.20 REQUIRED)

include_directories(../models)
include_directories(../control)
include_directories(../guidance)