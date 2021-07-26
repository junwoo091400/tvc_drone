# TVC Drone
ROS workspace containing the packages and tools for the drone TVC prototype

## Description

### Packages
- **drone_gnc**: Contains the GNC algorithms to control the drone. This is the only package that needs to be copied on the Raspberry Pi for real tests.
- **drone_utils**: Provides many utilities, as well as the launch files to run the code.
- **real_time_simulator**: Real-time rocket simulator and GUI. See original repo for more information.
- **drone_simulator_interface**: Interface to use the real time simulator with the drone. Transforms drone topics to rocket topics and vice versa.
- **rqt_ez_publisher**: External tool used in the GUI.

Each package contains its own readme file with more information.

## Getting Started

### Dependencies

* See the real_time_simulator installation instructions

### Installing

* Clone the content of the repo into your src folder (e. g. ~/drone_ws/src)

### Launching the program

* Build
```
cd ~/drone_ws
catkin_make
```

* Run in simulation
```
roslaunch drone_utils simu_drone.launch
```

* Run remotely on Raspberry Pi over Wi-Fi
```
roscd drone_utils/bash_scripts
./remote_drone_launch.sh
```

## Authors

Raphaël Linsen (raphael.linsen@epfl.ch)