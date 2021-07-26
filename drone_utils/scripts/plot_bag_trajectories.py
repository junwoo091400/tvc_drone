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

from math import acos

import seaborn as sns
sns.set()
# sns.set_style("whitegrid")
# blue, = sns.color_palette("muted", 1)


NX = 12
NU = 4

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/log.bag')

NNODE = 0
for topic, msg, t in bag.read_messages(topics=['/control/debug/horizon']):
    NNODE = len(msg.trajectory)
    break

def convert_state_to_array(state):
    q = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
    if np.isnan(q).any():
        q = [0, 0, 0, 1]
    r = R.from_quat(q)
    attitude_euler = r.as_euler('xyz', degrees=True)
    state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            attitude_euler[0], attitude_euler[1], attitude_euler[2],
                            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z])
    return state_array


def convert_control_to_array(control):
    control_array = np.array([control.servo1, control.servo2, control.bottom, control.top])
    return control_array


for topic, msg, t in bag.read_messages(topics=['/optitrack_state']):

    # print initial optitrack
    state = msg
    r = R.from_quat([state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w])
    omega_body = np.array([state.twist.angular.x, state.twist.angular.y, state.twist.angular.z])
    omega_inertial = r.apply(omega_body)
    state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
                            omega_inertial[0], omega_inertial[1], omega_inertial[2]])
    print(state_array)
    break
time_init = 0
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    time_init = t.to_sec()
    break
t_end = time_init + 30

kalman_state_history = np.empty((0, NX + 1))
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        kalman_state_history = np.vstack((kalman_state_history, state_array))


optitrack_state_history = np.empty((0, NX + 1))
for topic, msg, t in bag.read_messages(topics=['/optitrack_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        optitrack_state_history = np.vstack((optitrack_state_history, state_array))

integrator_state_history = np.empty((0, NX + 1))
for topic, msg, t in bag.read_messages(topics=['/simu_drone_state']):
    # print(msg)
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
i = 0
for topic, msg, t in bag.read_messages(topics=['/control/debug/horizon']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_horizon = np.empty((0, NX + 1))
        control_horizon = np.empty((0, NU + 1))
        for waypoint_stamped in msg.trajectory:
            x = waypoint_stamped.state.pose.orientation.x
            y = waypoint_stamped.state.pose.orientation.y
            z = waypoint_stamped.state.pose.orientation.z
            w = waypoint_stamped.state.pose.orientation.w
            # if i==1:
            #     print(x, y, z, w)
            #     print(z**2 + w**2 -x**2 - y**2)
                # print(acos(z**2 + w**2 -x**2 - y**2)*180/math.pi)

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
        i += 1
        state_horizon_history = np.dstack((state_horizon_history, state_horizon))
        control_horizon_history = np.dstack((control_horizon_history, control_horizon))

# fig, axe = plt.subplots(5, 3, figsize=(15, 10))
fig, axe = plt.subplots(4, 3, figsize=(15, 10))
fig.subplots_adjust(wspace=0.4, hspace=0.5)

var_indexes = {
    "t": 0,
    "x": 1,
    "y": 2,
    "z": 3,
    "dx": 4,
    "dy": 5,
    "dz": 6,
    "yaw (x)": 7,
    "pitch (y)": 8,
    "roll (z)": 9,
    "dyaw (x)": 10,
    "dpitch (y)": 11,
    "droll (z)": 12,
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

    (2, 0): ("t", "yaw (x)"),
    (2, 1): ("t", "pitch (y)"),
    (2, 2): ("t", "roll (z)"),

    (3, 0): ("t", "dyaw (x)"),
    (3, 1): ("t", "dpitch (y)"),
    (3, 2): ("t", "droll (z)"),


    # (0, 0): ("y", "z"),
}

control_plot_indexes = {
    # (4, 0): ("t", "servo1"),
    # (4, 0): ("t", "servo2"),
    # (4, 1): ("t", "bottom"),
    # (4, 2): ("t", "top"),
}

plot_ranges = {
    "t": [0, t_end - time_init],
    "x": [-3, 3],
    "y": [-3, 3],
    "z": [-1, 40],
    "dx": [-1.5, 1.5],
    "dy": [-1.5, 1.5],
    "dz": [-1.5, 5],
    "yaw (x)": [-15, 15],
    "pitch (y)": [-15, 15],
    "roll (z)": [-90, 90],
    "dyaw (x)": [-2, 2],
    "dpitch (y)": [-2, 2],
    "droll (z)": [-2, 2],
    "servo1": [-0.15, 0.15],
    "servo2": [-0.15, 0.15],
    "bottom": [0, 100],
    "top": [0, 100],
}

# axe[(0, 0)].set_aspect('equal', adjustable='box')

# set plot ranges
for plot_idx, (x_name, y_name) in state_plot_indexes.iteritems():
    axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                       xmax=plot_ranges[x_name][1],
                       ymin=plot_ranges[y_name][0],
                       ymax=plot_ranges[y_name][1])
    axe[plot_idx].set_xlabel(x_name)
    axe[plot_idx].set_ylabel(y_name)

for plot_idx, (x_name, y_name) in control_plot_indexes.iteritems():
    axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                       xmax=plot_ranges[x_name][1],
                       ymin=plot_ranges[y_name][0],
                       ymax=plot_ranges[y_name][1])
    axe[plot_idx].set_xlabel(x_name)
    axe[plot_idx].set_ylabel(y_name)


def plot_history(history, plot_indexes, axe, name, *plt_args):
    if history.shape[0] == 0:
        return
    line_list = []

    for plot_idx, (x_name, y_name) in plot_indexes.iteritems():
        x_data = history[:, var_indexes[x_name]]
        y_data = history[:, var_indexes[y_name]]
        line, = axe[plot_idx].plot(x_data, y_data, label=name, *plt_args)
        line_list.append((line, plot_idx))

    return line_list


for state_history, name in zip([kalman_state_history], ["state"]):
    plot_history(state_history, state_plot_indexes, axe, name)


plot_history(control_history, control_plot_indexes, axe, name)


def plot_horizon_segment(t):
    time_val = t
    time_array = state_horizon_history[0, 0, :]
    idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
    state_history = state_horizon_history[:, :, idx]
    control_history = control_horizon_history[:, :, idx]

    for plot_idx, (x_name, y_name) in state_plot_indexes.iteritems():
        x_data = state_history[:, var_indexes[x_name]]
        y_data = state_history[:, var_indexes[y_name]]
        line, = axe[plot_idx].plot(x_data, y_data, 'g-', label='_nolegend_')

PLOT_HORIZON_SEGMENTS = True

if PLOT_HORIZON_SEGMENTS:
    for t in np.arange(1.4, t_end-time_init, 1):
        plot_horizon_segment(t)
else:
    control_mpc_line_list = plot_history(control_horizon_history[:, :, 0], control_plot_indexes, axe, "mpc horizon", 'g-')
    state_mpc_line_list = plot_history(state_horizon_history[:, :, 0], state_plot_indexes, axe, "mpc horizon", 'g-')

    # control_mpc_line_list = []
    # state_mpc_line_list = []

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

axe[0][0].legend(loc='upper left')
plt.show()
