#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python

import rospy
import rospkg
from tvc_simulator.msg import FSM
from tvc_simulator.msg import DroneState
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor
from tvc_simulator.msg import Trajectory
from tvc_simulator.msg import Waypoint

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import time

import rosbag
import csv


def convert_state_to_array(state):
    state_array = np.array([state.pose.position.x, state.pose.position.y,  state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
                            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z])
    return state_array

def convert_control_to_array(control):
    state_array = np.array([control.servo1, control.servo2, control.bottom, control.top])
    return state_array

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('tvc_simulator')+'/log/log.bag')

time_init = 0
for topic, msg, t in bag.read_messages(topics=['/commands']):
    time_init = t.to_sec()
    break

t_end = time_init+1.5

kalman_state_history = np.empty((0, 14))
# save data to csv file to use in matlab
for topic, msg, t in bag.read_messages(topics=['/real_test_drone_state']):
    if t.to_sec()>time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec()-time_init, convert_state_to_array(msg))
        kalman_state_history = np.vstack((kalman_state_history, state_array))

kalman2_state_history = np.empty((0, 14))
# save data to csv file to use in matlab
for topic, msg, t in bag.read_messages(topics=['/kalman_drone_state']):
    if t.to_sec()>time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec()-time_init, convert_state_to_array(msg))
        kalman2_state_history = np.vstack((kalman2_state_history, state_array))

integrator_state_history = np.empty((0, 14))
# save data to csv file to use in matlab
for topic, msg, t in bag.read_messages(topics=['/rocket_state2']):
    if t.to_sec()>time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec()-time_init, convert_state_to_array(msg))
        integrator_state_history = np.vstack((integrator_state_history, state_array))


basic_kalman_state_history = np.empty((0, 14))
# save data to csv file to use in matlab
for topic, msg, t in bag.read_messages(topics=['/basic_kalman_drone_state']):
    if t.to_sec()>time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec()-time_init, convert_state_to_array(msg))
        basic_kalman_state_history = np.vstack((basic_kalman_state_history, state_array))



fig, axe = plt.subplots(4,4, figsize=(15,10))
import sys


for state_history, name in zip([kalman_state_history, kalman2_state_history, integrator_state_history, basic_kalman_state_history], ["real", "kal", "int", "sens"]):
    t_array = state_history[:, 0]
    
    axe[0][0].plot(t_array, state_history[:, 1], label="x"+" "+name)

    axe[0][1].plot(t_array, state_history[:, 2], label="y"+" "+name)
    axe[0][2].plot(t_array, state_history[:, 3], label="z"+" "+name)
    
    axe[1][0].plot(t_array, state_history[:, 4], label="dx"+" "+name)
    axe[1][1].plot(t_array, state_history[:, 5], label="dy"+" "+name)
    axe[1][2].plot(t_array, state_history[:, 6], label="dz"+" "+name)
    # axe[1][0].legend(loc="upper right")
    
    axe[2][0].plot(t_array, state_history[:, 7], label="q_x"+" "+name)
    axe[2][1].plot(t_array, state_history[:, 8], label="q_y"+" "+name)
    axe[2][2].plot(t_array, state_history[:, 9], label="q_z"+" "+name)
    axe[2][3].plot(t_array, state_history[:, 10], label="q_w"+" "+name)
    # axe[0][1].legend(loc="upper right")
    
    axe[3][0].plot(t_array, state_history[:, 11], label="dyaw (x)"+" "+name)
    axe[3][1].plot(t_array, state_history[:, 12], label="dpitch (y)"+" "+name)
    axe[3][2].plot(t_array, state_history[:, 13], label="droll (z)"+" "+name)
    # axe[1][1].legend(loc="upper right")

for i in range(4):
    for j in range(4):
        axe[i][j].legend(loc="upper left")


#
# t, bottom, top = np.loadtxt('input.csv', delimiter=',', unpack=True)
# l = axe[0][1].plot(t, bottom,)
# axe[0][1].legend(l, ('bottom'))
#
# l = axe[1][1].plot(t, top)
# axe[1][1].legend(l, ('top'))
#
plt.show()
