cmake_minimum_required(VERSION 3.13)
project(drone)

set(CMAKE_MODULE_PATH /usr/local/share/cmake/Modules/)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OsqpEigen)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(CYGWIN OR WIN32)
#FOR WINDOWS COMPILATION TO BE ABLE TO COMPILE LARGE LIBRARIES 
message(STATUS ">>> WINDOWS OS")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj -O3 -pg -Wall -Xassembler -mbig-obj -Wno-unused-variable ")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wa,-mbig-obj -O3 -pg -Wall -Werror -Xassembler -mbig-obj -Wno-unused-variable ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")
else()
#FOR LINUX AND APPLE COMPILATION
message(STATUS ">>> UNIX OS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")
endif()
set(CMAKE_VERBOSE_MAKEFILE “ON”)
set(CMAKE_BUILD_TYPE "Release")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-tautological-compare -Wno-enum-compare -Wno-sign-compare")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-tautological-compare -Wno-enum-compare -Wno-sign-compare")
endif ()

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=1000000) # 1 MB - For both Windows and UNIX
#set(CMAKE_CXX_FLAGS "-DEIGEN_STACK_ALLOCATION_LIMIT=100000000") 
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=0")

include_directories(../submodule/polympc/src)

add_subdirectory(control)
add_subdirectory(guidance)
add_subdirectory(models)
add_subdirectory(navigation)
add_subdirectory(tests)


