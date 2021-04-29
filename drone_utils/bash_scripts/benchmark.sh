#!/bin/bash

(roslaunch drone_utils drone_benchmark.launch &>/dev/null)&
sleep 1
rosrun drone_utils benchmark.py

#sleep 5
rosnode kill -a >/dev/null
sleep 2
rosrun drone_utils plot_bag_trajectories.py