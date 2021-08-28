#!/usr/bin/env python3
import rospy

import numpy as np
import math
import serial
import time
import matplotlib
import matplotlib.pyplot as plt

from drone_gnc.msg import DroneState
from geometry_msgs.msg import Vector3

from std_msgs.msg import String
from std_msgs.msg import Float64

computation_time_array = np.empty((0))
def saveComputationTime(computation_time):
    global computation_time_array
    computation_time_array = np.append(computation_time_array, computation_time.data)
    print "mpc:", computation_time.data, "ms"

# current_pos = Vector3()
# def stateCallback(state):
#     global current_pos

if __name__ == '__main__':
    # Init ROS
    rospy.init_node('benchmark', anonymous=True)

    # rospy.Subscriber("/drone_state", DroneState, stateCallback)

    command_pub = rospy.Publisher('/commands', String, queue_size=10)
    target_apogee_pub = rospy.Publisher('/target_apogee', Vector3, queue_size=10)

    rospy.Subscriber("/control/debug/computation_time", Float64, saveComputationTime)


    time.sleep(1)
    target_apogee_pub.publish(Vector3(1, 0, 0))
    command_pub.publish("Launch")

    start_time = rospy.get_time()

    BENCHMARK_DURATION = 4

    i = 0
    # Node rate in Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.get_time()-start_time < BENCHMARK_DURATION:
        rate.sleep()
    print "done benchmark"
    print "mean:", computation_time_array.mean()
    print "std:", computation_time_array.std()
    print "max:", computation_time_array.max()