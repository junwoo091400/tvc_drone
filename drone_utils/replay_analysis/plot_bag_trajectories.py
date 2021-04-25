#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from scipy.spatial.transform import Rotation as R
from matplotlib.widgets import Slider, Button, RadioButtons
import time
import rosbag
import csv


def convert_state_to_array(state):
    state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z,
                            state.pose.orientation.w,
                            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z])
    return state_array


def convert_control_to_array(control):
    state_array = np.array([control.servo1, control.servo2, control.bottom, control.top])
    return state_array


rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/log.bag')

time_init = 0
for topic, msg, t in bag.read_messages(topics=['/commands']):
    time_init = t.to_sec()
    break

t_end = time_init + 8

kalman_state_history = np.empty((0, 14))
# save data to csv file to use in matlab
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        kalman_state_history = np.vstack((kalman_state_history, state_array))

integrator_state_history = np.empty((0, 14))
for topic, msg, t in bag.read_messages(topics=['/simu_drone_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        integrator_state_history = np.vstack((integrator_state_history, state_array))

horizon_history = np.empty((11, 14, 0))
# save data to csv file to use in matlab
for topic, msg, t in bag.read_messages(topics=['/control/debug/horizon']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        horizon = np.empty((0, 14))
        for state_stamped in msg.trajectory:
            state_array = np.append(state_stamped.header.stamp.to_sec() - time_init,
                                    convert_state_to_array(state_stamped.state))
            horizon = np.vstack((horizon, state_array))
        horizon_history = np.dstack((horizon_history, horizon))

fig, axe = plt.subplots(4, 4, figsize=(15, 10))

var_indexes = {
    "t": 0,
    "x": 1,
    "y": 2,
    "z": 3,
    "dx": 4,
    "dy": 5,
    "dz": 6,
    "q_x": 7,
    "q_y": 8,
    "q_z": 9,
    "q_w": 10,
    "dyaw (x)": 11,
    "dpitch (y)": 12,
    "droll (z)": 13,
}

plot_indexes = {
    (0, 0): ("t", "x"),
    (0, 1): ("t", "y"),
    (0, 2): ("t", "z"),

    (1, 0): ("t", "dx"),
    (1, 1): ("t", "dy"),
    (1, 2): ("t", "dz"),

    (2, 0): ("t", "q_x"),
    (2, 1): ("t", "q_y"),
    (2, 2): ("t", "q_z"),
    (2, 3): ("t", "q_w"),

    (3, 0): ("t", "dyaw (x)"),
    (3, 1): ("t", "dpitch (y)"),
    (3, 2): ("t", "droll (z)"),
}

plot_ranges = {
    "t": [0, t_end - time_init],
    "x": [-2, 2],
    "y": [-2, 2],
    "z": [-2, 2],
    "dx": [-2, 2],
    "dy": [-2, 2],
    "dz": [-2, 2],
    "q_x": [-0.3, 0.3],
    "q_y": [-0.3, 0.3],
    "q_z": [-0.3, 0.3],
    "q_w": [0.7, 1.3],
    "dyaw (x)": [-0.3, 0.3],
    "dpitch (y)": [-0.3, 0.3],
    "droll (z)": [-0.3, 0.3],
}

# set plot ranges
for plot_idx, (x_name, y_name) in plot_indexes.iteritems():
    axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                       xmax=plot_ranges[x_name][1],
                       ymin=plot_ranges[y_name][0],
                       ymax=plot_ranges[y_name][1])


def plot_state_history(state_history, axe, name):
    line_list = []

    for plot_idx, (x_name, y_name) in plot_indexes.iteritems():
        x_data = state_history[:, var_indexes[x_name]]
        y_data = state_history[:, var_indexes[y_name]]
        line_list.append((axe[plot_idx].plot(x_data, y_data)[0], plot_idx))

    return line_list


for state_history, name in zip([integrator_state_history, kalman_state_history], ["real", "kal"]):
    plot_state_history(state_history, axe, name)

mpc_line_list = plot_state_history(horizon_history[:, :, 0], axe, "mpc")

plt.subplots_adjust(left=0.15, bottom=0.25)
ax_slider = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='white')
slider = Slider(ax_slider, 'Time', 0, t_end-time_init, valinit=0, valstep=0.05)


def update(val):
    time_val = slider.val
    time_array = horizon_history[0, 0, :]
    idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
    state_history = horizon_history[:, :, idx]
    t_array = state_history[:, 0]

    for i in range(len(mpc_line_list)):
        l, plot_idx = mpc_line_list[i]
        x_name, y_name = plot_indexes[plot_idx]
        l.set_ydata(state_history[:, var_indexes[y_name]])
        l.set_xdata(state_history[:, var_indexes[x_name]])

    fig.canvas.draw_idle()


slider.on_changed(update)

plt.show()
