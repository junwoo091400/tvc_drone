#!/bin/bash

(ssh drone@ert.local 'isolcpus=1,2,3')&

# start the master on the raspberry pi
(ssh drone@ert.local 'source /home/drone/drone_ws/devel/setup.bash;roscore')&

# copy the necessay files on the raspberrypi
./copy_on_pi.sh

# compile on the raspberrypi if the argument "-c" is given
if [[ $1 == "-c" ]]; then
    ssh drone@ert.local 'source /home/drone/drone_ws/devel/setup.bash;cd ~/drone_ws;catkin_make'
fi

# (ssh drone@ert.local 'sudo cpufreq-set -g performance')&


export ROS_IP=$(hostname -I | cut -d' ' -f1) #automatically set to local ip address
export ROS_MASTER_URI=http://$(getent hosts ert.local | awk '{ print $1 }'):11311 #automatically set to raspberry pi ip address

roslaunch --wait drone_utils PIL.launch
