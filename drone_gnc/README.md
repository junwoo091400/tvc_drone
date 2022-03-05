# Drone GNC
GNC algorithms for controlling the drone TVC prototype

## Modules
- **guidance**: Nonlinear MPC to find the optimal trajectory to the target apogee
- **navigation**: Extended Kalman Filter
- **control**: Nonlinear MPC controller that tracks the guidance trajectory
- **models**: Contains the drone model used by the MPC controller and the kalman filter
- **ros_nodes**: ROS interfaces for the different modules
  - **utils/drone_fsm_node**: State machine
  - **utils/drone_mavros_interface**: Interface to communicate with the Pixhawk using MAVROS (uses MAVLink over serial)
  - **utils/drone_dome**: Contains the VPRN interface to access the Optitrack pose data in the drone dome
- **tests**: Contains tests to debug the code without ROS. Use the _CmakeLists.txt_ located inside of the _src_ directory instead of the one in _drone_gnc_ that uses ROS.