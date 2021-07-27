# Drone GNC
GNC algorithms for controlling the drone TVC prototype

## Modules
- **guidance**: Nonlinear MPC to find the optimal trajectory to the target apogee
- **navigation**: Extended Kalman Filter
- **control**: Nonlinear MPC controller that tracks the guidance trajectory
- **models**: contains the drone model used by the MPC controller and the kalman filter
- **utils**:
    - drone_fsm_node: state machine
    - drone_dome: contains the VPRN interface to access the Optitrack pose data in the drone dome
    - control_devel: script used for debugging (use drone_gnc.launch)
  