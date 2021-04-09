#!/bin/bash
(ssh pi@raspberrypi.local 'source /home/pi/catkin_ws/devel/setup.bash;roscore')&
cd ../onboard_interface/drone
./copy_on_pi.sh

if [[ $1 == "c" ]]; then
    ssh pi@raspberrypi.local 'source /home/pi/catkin_ws/devel/setup.bash;cd ~/catkin_ws;catkin_make'
fi


export ROS_IP=192.168.0.186
export ROS_MASTER_URI=http://192.168.0.196:11311

roslaunch tvc_simulator real_drone.launch
