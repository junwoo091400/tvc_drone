# Drone Optimal Control
Optimal control-based algorithms for controlling the drone TVC prototype

## Modules
- **guidance**: Free terminal time, minimal-energy optimization to find the optimal trajectory to the target apogee
- **control**: Nonlinear MPC controller that tracks the guidance trajectory
- **models**: Contains the drone model used by the MPC controller and the kalman filter
- **ros_nodes**: ROS wrappers for the different modules
- **tests**: Contains tests to debug the code without ROS.