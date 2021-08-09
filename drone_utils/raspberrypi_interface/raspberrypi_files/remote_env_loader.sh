#!/bin/bash

export ROS_IP=$(hostname -I | cut -d' ' -f1) #automatically set to local ip address
export ROS_MASTER_URI=http://$(hostname -I | cut -d' ' -f1):11311 #automatically set to local ip address
export ROSLAUNCH_SSH_UNKNOWN=1

source /home/drone/drone_ws/devel/setup.bash

exec "$@"
