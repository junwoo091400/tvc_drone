#!/bin/bash
(roslaunch tvc_simulator launch_replay_nodes.launch)&
sleep 1
rosbag play $(rospack find tvc_simulator)/postProcess/replay_analysis/test_02_04_21_bags/bags/log4_f_d.bag /kalman_drone_state:=/real_test_drone_state
rosnode kill -a
sleep 3
rosrun tvc_simulator plot_replay_results.py