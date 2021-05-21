#!/bin/bash
(roslaunch drone_utils replay_nodes.launch)&
sleep 3
rosbag play $(rospack find drone_utils)/replay_analysis/best_data/log2_airborne.bag /drone_state:=/optitrack_state /simu_drone_state:=/null /rocket_state:=/null
rosnode kill -a
sleep 3
rosrun drone_utils plot_bag_trajectories.py
#roslaunch drone_utils rosbag_replay.launch