#!/bin/bash
#copy avionic bridge
rsync -avuzh on_pi/avionic_bridge.py pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/scripts/avionic_bridge.py

#copy actuators_tests
rsync -avuzh on_pi/actuators_tests.py pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/scripts/actuators_tests.py

#copy launch file
#rsync -avuzh on_pi/drone_pi_launch.launch pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/drone_pi_launch.launch

#copy drone control c++ files
rsync -avuzh ../../src/prototypes/drone/* pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/src
rsync -avuzh ../../src/gnc/rocket_model.hpp pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/src
rsync -avuzh ../../src/gnc/control/polympc_redef.hpp pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/src

#copy config files
rsync -avuzh ../../config/prototypes/drone/* pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/config

#copy time_keeper
rsync -avuzh ../../src/simulator/time_keeper.cpp pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/src

rsync -avuzh  ../../msg/* pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/msg
rsync -avuzh  ../../srv/* pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator/srv

#copy CMakeLists
rsync -avuzh on_pi/CMakeLists.txt  pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator


#copy env loader
rsync -avuzh on_pi/remote_env_loader.sh  pi@raspberrypi.local:~/catkin_ws/devel

#copy PolyMPC
#rsync -avuzh  ../../submodule  pi@raspberrypi.local:~/catkin_ws/src/drone_tvc_simulator
