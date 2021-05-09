#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import time
import matplotlib
import matplotlib.pyplot as plt

from drone_gnc.msg import DroneControl
from drone_gnc.msg import FSM
from drone_gnc.msg import DroneState
from drone_gnc.msg import Sensor
from drone_gnc.msg import Trajectory
from drone_gnc.msg import Waypoint
from geometry_msgs.msg import Vector3

from std_msgs.msg import String
from std_msgs.msg import Float64
from drone_gnc.srv import GetFSM
from drone_gnc.srv import InterpolateControlSpline

trajectory = np.array([
    Vector3(0, 0, 1),
    Vector3(-0.4, 0, 0.3),
    Vector3(-0.8, 0, 1),
    Vector3(-0.8, 0, 0),
])

computation_time_array = np.empty((0))


def saveComputationTime(computation_time):
    global computation_time_array
    computation_time_array = np.append(computation_time_array, computation_time.data)


current_pos = Vector3()


def stateCallback(state):
    global current_pos
    current_pos = state.pose.position


if __name__ == '__main__':
    # Init ROS
    rospy.init_node('benchmark', anonymous=True)

    rospy.Subscriber("/drone_state", DroneState, stateCallback)

    rospy.wait_for_service('/getFSM_gnc')
    client_fsm = rospy.ServiceProxy('/getFSM_gnc', GetFSM)

    command_pub = rospy.Publisher('/commands', String, queue_size=10)
    target_apogee_pub = rospy.Publisher('/target_apogee', Vector3, queue_size=10)

    rospy.Subscriber("/control/debug/computation_time", Float64, saveComputationTime)

    i = 0

    time.sleep(0.5)
    target_pos = trajectory[i]
    target_apogee_pub.publish(target_pos)
    time.sleep(3)
    command_pub.publish("Launch")

    start_time = rospy.get_time()

    BENCHMARK_DURATION = 3

    # Node rate in Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and i < len(trajectory):
        x1 = current_pos.x
        y1 = current_pos.y
        z1 = current_pos.z
        x2 = target_pos.x
        y2 = target_pos.y
        z2 = target_pos.z
        if ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5 < 0.1:
            i += 1
            target_pos = trajectory[i]
            target_apogee_pub.publish(target_pos)

        rate.sleep()
    print "done benchmark"
    print "mean:", computation_time_array.mean()
    print "std:", computation_time_array.std()
    print "max:", computation_time_array.max()
