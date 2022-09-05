cmake_minimum_required(VERSION 3.0.2)
project(tests)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

# matplotlib_cpp library
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
  )
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
  )
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
    )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)

add_executable(control_mpc_test control_mpc_test.cpp)
target_link_libraries(control_mpc_test drone_mpc Eigen3::Eigen OsqpEigen::OsqpEigen ${PYTHON_LIBRARIES} matplotlib_cpp)

add_executable(control_mpc_simulation_test control_mpc_simulation_test.cpp)
target_link_libraries(control_mpc_simulation_test drone_mpc Eigen3::Eigen OsqpEigen::OsqpEigen ${PYTHON_LIBRARIES})

add_executable(guidance_test guidance_test.cpp)
target_link_libraries(guidance_test drone_guidance Eigen3::Eigen OsqpEigen::OsqpEigen matplotlib_cpp)

add_executable(terminal_cost_test terminal_cost_test.cpp)
target_link_libraries(terminal_cost_test mpc_utils Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(full_trajectory_tracking_test full_trajectory_tracking_test.cpp)
target_link_libraries(full_trajectory_tracking_test drone_mpc drone_guidance Eigen3::Eigen OsqpEigen::OsqpEigen)