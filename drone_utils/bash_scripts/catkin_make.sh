#!/bin/bash
set -e
cd ~/drone_ws
source /opt/ros/melodic/setup.bash
source ~/drone_ws/devel/setup.bash
export PATH=/snap/bin:$PATH
catkin_make -j8

