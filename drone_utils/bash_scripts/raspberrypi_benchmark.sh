#!/bin/bash
# start the master on the raspberry pi
(ssh pi@raspberrypi.local 'source /home/pi/drone_ws/devel/setup.bash;roscore')&

# copy the necessay files on the raspberrypi
./copy_on_pi.sh

# compile on the raspberrypi if the argument "c" is given
if [[ $1 == "c" ]]; then
    ssh pi@raspberrypi.local 'source /home/pi/drone_ws/devel/setup.bash;cd ~/drone_ws;catkin_make'
fi

export ROS_IP=$(hostname -I | cut -d' ' -f1) #automatically set to local ip address
export ROS_MASTER_URI=http://$(getent hosts raspberrypi.local | awk '{ print $1 }'):11311 #automatically set to raspberry pi ip address

# roslaunch drone_utils raspberrypi_benchmark.launch

(roslaunch drone_utils raspberrypi_benchmark.launch &>/dev/null)&
sleep 1
rosrun drone_utils benchmark.py

#sleep 5
rosnode kill -a >/dev/null
sleep 2
rosrun drone_utils plot_bag_trajectories.py