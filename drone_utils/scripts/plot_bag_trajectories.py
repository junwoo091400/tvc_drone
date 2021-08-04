#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag
import seaborn as sns

sns.set()

from plot_utils import convert_state_to_array, convert_control_to_array, NP, NX, NU, var_indexes, plot_history, read_state_history, read_control_history, read_horizon_history

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/log.bag')


time_init = 0
t_end = 0
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if time_init == 0:
        time_init = t.to_sec()
    t_end = t.to_sec()
# t_end = time_init + 30

kalman_state_history = read_state_history(bag, '/drone_state', time_init, t_end)
integrator_state_history = read_state_history(bag, '/simu_drone_state', time_init, t_end)
state_horizon_history, control_horizon_history = read_horizon_history(bag, '/control/debug/horizon', time_init, t_end)
control_history = read_control_history(bag, '/drone_control', time_init, t_end)
guidance_state_horizon_history, guidance_control_horizon_history = read_horizon_history(bag, '/guidance/horizon', time_init, t_end)

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

    (4, 0): ("t", "bottom"),
    (4, 1): ("t", "top"),
    (4, 2): ("t", "servo1"),

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

    "thrust_scaling": [0, 1.5],
    "torque_scaling": [0, 1.5],
    "servo1_offset": [-0.1, 0.1],
    "servo2_offset": [-0.1, 0.1],
    "fx": [-1, 1],
    "fy": [-1, 1],
    "fz": [-1, 1],
    "mx": [-1, 1],
    "my": [-1, 1],
    "mz": [-1, 1],
}

fig, axe = plt.subplots(5, 3, figsize=(15, 10))
fig.subplots_adjust(wspace=0.4, hspace=0.5)

# set plot ranges
for plot_idx, (x_name, y_name) in state_plot_indexes.items() + control_plot_indexes.items():
    axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                       xmax=plot_ranges[x_name][1],
                       ymin=plot_ranges[y_name][0],
                       ymax=plot_ranges[y_name][1])
    axe[plot_idx].set_xlabel(x_name)
    axe[plot_idx].set_ylabel(y_name)

plot_history(kalman_state_history, state_plot_indexes, axe, "state")

plot_history(control_history, control_plot_indexes, axe, "control")


def plot_horizon_segment(t):
    time_val = t
    time_array = state_horizon_history[0, 0, :]
    idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
    state_history = state_horizon_history[:, :, idx]
    control_history = control_horizon_history[:, :, idx]

    for plot_idx, (x_name, y_name) in state_plot_indexes.items():
        x_data = state_history[:, var_indexes[x_name]]
        y_data = state_history[:, var_indexes[y_name]]
        line, = axe[plot_idx].plot(x_data, y_data, 'g-', label='_nolegend_')


PLOT_HORIZON_SEGMENTS = False

if PLOT_HORIZON_SEGMENTS:
    for t in np.arange(1.4, t_end - time_init, 1):
        plot_horizon_segment(t)
else:
    if control_horizon_history.size != 0:
        control_mpc_line_list = plot_history(control_horizon_history[:, :, 0], control_plot_indexes, axe, "mpc horizon",
                                             'g-')
    if state_horizon_history.size != 0:
        state_mpc_line_list = plot_history(state_horizon_history[:, :, 0], state_plot_indexes, axe, "mpc horizon", 'b-')

    if guidance_control_horizon_history.size != 0:
        guidance_control_mpc_line_list = plot_history(guidance_control_horizon_history[:, :, 0], control_plot_indexes, axe, "guidance traj",
                                             'g-')
    if guidance_state_horizon_history.size != 0:
        guidance_state_mpc_line_list = plot_history(guidance_state_horizon_history[:, :, 0], state_plot_indexes, axe, "guidance traj", 'y-')

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

        time_array = guidance_state_horizon_history[0, 0, :]
        idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
        state_history = guidance_state_horizon_history[:, :, idx]
        control_history = guidance_control_horizon_history[:, :, idx]

        for i in range(len(guidance_state_mpc_line_list)):
            l, plot_idx = guidance_state_mpc_line_list[i]
            x_name, y_name = state_plot_indexes[plot_idx]
            l.set_ydata(state_history[:, var_indexes[y_name]])
            l.set_xdata(state_history[:, var_indexes[x_name]])

        for i in range(len(guidance_control_mpc_line_list)):
            l, plot_idx = guidance_control_mpc_line_list[i]
            x_name, y_name = control_plot_indexes[plot_idx]
            l.set_ydata(control_history[:, var_indexes[y_name]])
            l.set_xdata(control_history[:, var_indexes[x_name]])

        fig.canvas.draw_idle()


    # print(control_horizon_history[:, :, 100])

    slider.on_changed(update)


axe[0][0].legend(loc='upper left')

kalman_plot_indexes = {
    (0, 0): ("t", "dx"),
    (0, 1): ("t", "dy"),
    (0, 2): ("t", "dz"),

    (1, 0): ("t", "dyaw (x)"),
    (1, 1): ("t", "dpitch (y)"),
    (1, 2): ("t", "droll (z)"),

    (2, 0): ("t", "thrust_scaling"),
    (2, 1): ("t", "servo1_offset"),
    (2, 2): ("t", "servo2_offset"),

    (3, 0): ("t", "mx"),
    (3, 1): ("t", "my"),
    (3, 2): ("t", "mz"),

    (4, 0): ("t", "fx"),
    (4, 1): ("t", "fy"),
    (4, 2): ("t", "fz"),
}

fig_kalman, axe_kalman = plt.subplots(5, 3, figsize=(20, 10))

plot_history(kalman_state_history, kalman_plot_indexes, axe_kalman, "kalman_state")

axe_kalman[0][0].legend(loc='upper left')

# set plot ranges
for plot_idx, (x_name, y_name) in kalman_plot_indexes.items():
    axe_kalman[plot_idx].axis(xmin=plot_ranges[x_name][0],
                              xmax=plot_ranges[x_name][1],
                              ymin=plot_ranges[y_name][0],
                              ymax=plot_ranges[y_name][1])
    axe_kalman[plot_idx].set_xlabel(x_name)
    axe_kalman[plot_idx].set_ylabel(y_name)

import rospy

fig_compute_times, axe_compute_times = plt.subplots(1, 3, figsize=(10, 5))
fig_compute_times.suptitle('Computation times [ms]', fontsize=16)

control_period = rospy.get_param('/control/mpc/mpc_period')*1000
guidance_period = rospy.get_param('/guidance/mpc/mpc_period')*1000
kalman_period = rospy.get_param('/navigation/period')*1000

control_computation_time_history = np.empty((0, 2))
for topic, msg, t in bag.read_messages(topics=['/control/debug/computation_time']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        control_computation_time_history = np.vstack((control_computation_time_history, np.array([t.to_sec()-time_init, msg.data])))

axe_compute_times[0].plot(control_computation_time_history[:, 0], control_computation_time_history[:, 1])
axe_compute_times[0].hlines(control_period, 0, t_end-time_init, 'r')
axe_compute_times[0].set_title('control')

guidance_computation_time_history = np.empty((0, 2))
for topic, msg, t in bag.read_messages(topics=['/guidance/debug/computation_time']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        guidance_computation_time_history = np.vstack((guidance_computation_time_history, np.array([t.to_sec()-time_init, msg.data])))

axe_compute_times[1].plot(guidance_computation_time_history[:, 0], guidance_computation_time_history[:, 1])
axe_compute_times[1].hlines(guidance_period, 0, t_end-time_init, 'r')
axe_compute_times[1].set_title('guidance')


guidance_computation_time_history = np.empty((0, 2))
for topic, msg, t in bag.read_messages(topics=['/navigation/debug/computation_time']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        guidance_computation_time_history = np.vstack((guidance_computation_time_history, np.array([t.to_sec()-time_init, msg.data])))

axe_compute_times[2].plot(guidance_computation_time_history[:, 0], guidance_computation_time_history[:, 1])
axe_compute_times[2].hlines(kalman_period, 0, t_end-time_init, 'r')
axe_compute_times[2].set_title('navigation')


plt.show()
