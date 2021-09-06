#!/bin/bash
# cut a rosbag and remove "tf" and "rocket_visualization" topics
# provide input bag name, start_time, end_time
# echo $1, $2, $3
t0=`rosbag info -y -k start $1.bag`
t_start=`echo "$t0 + $2" | bc -l`
t_end=`echo "$t0 + $3" | bc -l`
# echo $t0, $t_start, $t_end
rosbag filter $1.bag $1_f.bag "t.secs >= $t_start and t.secs <= $t_end and topic != '/tf' and topic != '/rocket_visualization'"
