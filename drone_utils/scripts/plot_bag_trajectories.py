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

NX = 13
NU = 4
NNODE = 6


def convert_state_to_array(state):
    state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z,
                            state.pose.orientation.w,
                            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z])
    return state_array


def convert_control_to_array(control):
    control_array = np.array([control.servo1, control.servo2, control.bottom, control.top])
    return control_array


rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/log.bag')

time_init = 0
for topic, msg, t in bag.read_messages(topics=['/commands']):
    time_init = t.to_sec()
    break

t_end = time_init + 5

kalman_state_history = np.empty((0, NX + 1))
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        kalman_state_history = np.vstack((kalman_state_history, state_array))

integrator_state_history = np.empty((0, NX + 1))
for topic, msg, t in bag.read_messages(topics=['/simu_drone_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        integrator_state_history = np.vstack((integrator_state_history, state_array))

control_history = np.empty((0, NU + 1))
for topic, msg, t in bag.read_messages(topics=['/drone_control']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        control_array = np.append(t.to_sec() - time_init, convert_control_to_array(msg))
        control_history = np.vstack((control_history, control_array))

state_horizon_history = np.empty((NNODE, NX + 1, 0))
control_horizon_history = np.empty((NNODE, NU + 1, 0))
for topic, msg, t in bag.read_messages(topics=['/control/debug/horizon']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_horizon = np.empty((0, NX + 1))
        control_horizon = np.empty((0, NU + 1))
        for waypoint_stamped in msg.trajectory:
            state_array = np.concatenate((
                np.array([waypoint_stamped.header.stamp.to_sec() - time_init]),
                convert_state_to_array(waypoint_stamped.state)
            ))
            control_array = np.concatenate((
                np.array([waypoint_stamped.header.stamp.to_sec() - time_init]),
                convert_control_to_array(waypoint_stamped.control)
            ))
            state_horizon = np.vstack((state_horizon, state_array))
            control_horizon = np.vstack((control_horizon, control_array))
        state_horizon_history = np.dstack((state_horizon_history, state_horizon))
        control_horizon_history = np.dstack((control_horizon_history, control_horizon))

# control_history = np.empty((0, 14))
# for topic, msg, t in bag.read_messages(topics=['/drone_control']):
#     if t.to_sec() > time_init and t.to_sec() < t_end:
#         state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
#         integrator_state_history = np.vstack((integrator_state_history, state_array))

fig, axe = plt.subplots(5, 4, figsize=(15, 10))

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
    "servo1": 1,
    "servo2": 2,
    "bottom": 3,
    "top": 4,
}

state_plot_indexes = {
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

control_plot_indexes = {
    (4, 0): ("t", "servo1"),
    (4, 1): ("t", "servo2"),
    (4, 2): ("t", "bottom"),
    (4, 3): ("t", "top"),
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
    "servo1": [-0.15, 0.15],
    "servo2": [-0.15, 0.15],
    "bottom": [0, 100],
    "top": [0, 100],
}

# set plot ranges
for plot_idx, (x_name, y_name) in state_plot_indexes.iteritems():
    axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                       xmax=plot_ranges[x_name][1],
                       ymin=plot_ranges[y_name][0],
                       ymax=plot_ranges[y_name][1])

for plot_idx, (x_name, y_name) in control_plot_indexes.iteritems():
    axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                       xmax=plot_ranges[x_name][1],
                       ymin=plot_ranges[y_name][0],
                       ymax=plot_ranges[y_name][1])


def plot_history(history, plot_indexes, axe, name):
    line_list = []

    for plot_idx, (x_name, y_name) in plot_indexes.iteritems():
        x_data = history[:, var_indexes[x_name]]
        y_data = history[:, var_indexes[y_name]]
        line, = axe[plot_idx].plot(x_data, y_data)
        line_list.append((line, plot_idx))

    return line_list


for state_history, name in zip([integrator_state_history, kalman_state_history], ["real", "kal"]):
    plot_history(state_history, state_plot_indexes, axe, name)


plot_history(control_history, control_plot_indexes, axe, name)

control_mpc_line_list = plot_history(control_horizon_history[:, :, 0], control_plot_indexes, axe, "mpc")
state_mpc_line_list = plot_history(state_horizon_history[:, :, 0], state_plot_indexes, axe, "mpc")

plt.subplots_adjust(left=0.15, bottom=0.25)
ax_slider = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='white')
slider = Slider(ax_slider, 'Time', 0, t_end - time_init, valinit=0, valstep=0.01)


def update(val):
    time_val = slider.val
    time_array = state_horizon_history[0, 0, :]
    idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
    state_history = state_horizon_history[:, :, idx]
    control_history = control_horizon_history[:, :, idx]

    for i in range(len(state_mpc_line_list)):
        l, plot_idx = state_mpc_line_list[i]
        x_name, y_name = state_plot_indexes[plot_idx]
        l.set_ydata(state_history[:, var_indexes[y_name]])
        l.set_xdata(state_history[:, var_indexes[x_name]])

    for i in range(len(control_mpc_line_list)):
        l, plot_idx = control_mpc_line_list[i]
        x_name, y_name = control_plot_indexes[plot_idx]
        l.set_ydata(control_history[:, var_indexes[y_name]])
        l.set_xdata(control_history[:, var_indexes[x_name]])

    fig.canvas.draw_idle()

# print(control_horizon_history[:, :, 100])

slider.on_changed(update)

plt.show()
