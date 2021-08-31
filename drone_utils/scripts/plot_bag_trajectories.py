#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt

USE_LATEX = False

if USE_LATEX:
    plt.rc('font', family='serif', serif='cm10')
    plt.rc('text', usetex=True)
    plt.rc('text.latex', preamble=r'\usepackage{gensymb}\usepackage{amsmath}')

from matplotlib.widgets import Slider
import rosbag
import seaborn as sns

sns.set()

from plot_utils import convert_state_to_array, convert_control_to_array, NP, NX, NU, var_indexes, plot_history,\
    read_state_history, read_control_history, read_horizon_history, set_plot_ranges

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/log.bag')


time_init = 0
t_end = 0
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if time_init == 0:
        time_init = msg.header.stamp.to_sec()
    t_end = msg.header.stamp.to_sec()
# t_end = time_init + 30

kalman_state_history = read_state_history(bag, '/drone_state', time_init, t_end)
integrator_state_history = read_state_history(bag, '/simu_drone_state', time_init, t_end)
state_horizon_history, control_horizon_history = read_horizon_history(bag, '/control/debug/horizon', time_init, t_end)
control_history = read_control_history(bag, '/drone_control', time_init, t_end)
guidance_state_horizon_history, guidance_control_horizon_history = read_horizon_history(bag, '/guidance/horizon', time_init, t_end)


plot_ranges = {
    "t": [0, t_end - time_init],
    "x": [-1.5, 1.5],
    "y": [-1.5, 1.5],
    "z": [0, 2],
    "dx": [-1.5, 1.5],
    "dy": [-1.5, 1.5],
    "dz": [-1.5, 5],
    "yaw (x)": [-15, 15],
    "pitch (y)": [-15, 15],
    "roll (z)": [-90, 90],
    "dyaw (x)": [-20, 20],
    "dpitch (y)": [-20, 20],
    "droll (z)": [-4, 4],

    "servo1": [-15, 15],
    "servo2": [-15, 15],
    "bottom": [0, 100],
    "top": [0, 100],

    "thrust_scaling": [0, 1.5],
    "torque_scaling": [0, 1.5],
    "servo1_offset": [-3, 3],
    "servo2_offset": [-3, 3],
    "fx": [-6, 6],
    "fy": [-1, 1],
    "fz": [-1, 1],
    "mx": [-1, 1],
    "my": [-1, 1],
    "mz": [-1, 1],
}

state_plot_indexes = {
    (0, 0): [("t", "z")],
    (0, 1): [("t", "y")],
    (0, 2): [("t", "z")],

    (1, 0): [("t", "dx")],
    (1, 1): [("t", "dy")],
    (1, 2): [("t", "dz")],

    (2, 0): [("t", "yaw (x)")],
    (2, 1): [("t", "pitch (y)")],
    (2, 2): [("t", "roll (z)")],

    (3, 0): [("t", "dyaw (x)")],
    (3, 1): [("t", "dpitch (y)")],
    (3, 2): [("t", "droll (z)")],
}
control_plot_indexes = {
    (4, 0): [("t", "bottom"), ("t", "top")],
    (4, 1): [("t", "servo1")],
    (4, 2): [("t", "servo2")],
}

fig, axe = plt.subplots(5, 3, figsize=(15, 10))

# state_plot_indexes = {
#     (0, 0): [("y", "z")],
# }
# control_plot_indexes = {}
# fig, axe = plt.subplots(1, 1, figsize=(5, 2.5))
# axe = np.array([[axe]])
# axe[0,0].axis('scaled')

# state_plot_indexes = {
#     (0, 0): [("t", "x")],
#     (1, 0): [("t", "fx")],
#     (2, 0): [("t", "my")],
# }
# control_plot_indexes = {}
# fig, axe = plt.subplots(3, 1, figsize=(10, 5))
# axe = np.array([[plot] for plot in axe])

fig.subplots_adjust(wspace=0.4, hspace=0.5)
set_plot_ranges(axe, plot_ranges, state_plot_indexes.items(), USE_LATEX)
set_plot_ranges(axe, plot_ranges, control_plot_indexes.items(), USE_LATEX)


plot_history(kalman_state_history, state_plot_indexes, axe, r"EKF estimated state")

plot_history(control_history, control_plot_indexes, axe, "control")


def plot_horizon_segment(t):
    time_val = t
    time_array = state_horizon_history[0, 0, :]
    idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
    state_history = state_horizon_history[:, :, idx]
    control_history = control_horizon_history[:, :, idx]

    for plot_idx, name_list in state_plot_indexes.items():
        for (x_name, y_name) in name_list:
            if var_indexes[x_name] <= NX and var_indexes[y_name] <= NX:
                x_data = state_history[:, var_indexes[x_name]]
                y_data = state_history[:, var_indexes[y_name]]
                axe[plot_idx].scatter([x_data[0]], [y_data[0]], s=15, marker='x', alpha=0.2, c='k')
                axe[plot_idx].plot(x_data, y_data, 'g-', label='_nolegend_')


PLOT_HORIZON_SEGMENTS = True

if PLOT_HORIZON_SEGMENTS:
    for t in np.arange(1.4, t_end - time_init, 1.3):
        plot_horizon_segment(t)
    # for legend only
    axe[0, 0].plot([], [], 'g-', label='mpc horizons')
else:
    if control_horizon_history.size != 0:
        control_mpc_line_list = plot_history(control_horizon_history[:, :, 0], control_plot_indexes, axe, "mpc horizon",
                                             'g-')
    if state_horizon_history.size != 0:
        state_mpc_line_list = plot_history(state_horizon_history[:, :, 0], state_plot_indexes, axe, "mpc horizon", 'g-')

    if guidance_control_horizon_history.size != 0:
        guidance_control_mpc_line_list = plot_history(guidance_control_horizon_history[:, :, 0], control_plot_indexes, axe, "guidance traj",
                                             'y-')
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
            for (x_name, y_name) in state_plot_indexes[plot_idx]:
                if var_indexes[x_name] <= NX and var_indexes[y_name] <= NX:
                    l.set_ydata(state_history[:, var_indexes[y_name]])
                    l.set_xdata(state_history[:, var_indexes[x_name]])

        for i in range(len(control_mpc_line_list)):
            l, plot_idx = control_mpc_line_list[i]
            for (x_name, y_name) in control_plot_indexes[plot_idx]:
                if var_indexes[x_name] <= NX and var_indexes[y_name] <= NX:
                    l.set_ydata(control_history[:, var_indexes[y_name]])
                    l.set_xdata(control_history[:, var_indexes[x_name]])

        time_array = guidance_state_horizon_history[0, 0, :]
        idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)
        state_history = guidance_state_horizon_history[:, :, idx]
        control_history = guidance_control_horizon_history[:, :, idx]

        for i in range(len(guidance_state_mpc_line_list)):
            l, plot_idx = guidance_state_mpc_line_list[i]
            for (x_name, y_name) in state_plot_indexes[plot_idx]:
                if var_indexes[x_name] <= NX and var_indexes[y_name] <= NX:
                    l.set_ydata(state_history[:, var_indexes[y_name]])
                    l.set_xdata(state_history[:, var_indexes[x_name]])

        for i in range(len(guidance_control_mpc_line_list)):
            l, plot_idx = guidance_control_mpc_line_list[i]
            for (x_name, y_name) in control_plot_indexes[plot_idx]:
                if var_indexes[x_name] <= NX and var_indexes[y_name] <= NX:
                    l.set_ydata(control_history[:, var_indexes[y_name]])
                    l.set_xdata(control_history[:, var_indexes[x_name]])

        fig.canvas.draw_idle()

    slider.on_changed(update)


axe[0][0].legend(loc='upper left')

PLOT_FULL_STATE = False
if PLOT_FULL_STATE:
    kalman_plot_indexes = {
        (0, 0): [("t", "dx")],
        (0, 1): [("t", "dy")],
        (0, 2): [("t", "dz")],

        (1, 0): [("t", "dyaw (x)")],
        (1, 1): [("t", "dpitch (y)")],
        (1, 2): [("t", "droll (z)")],

        (2, 0): [("t", "thrust_scaling")],
        (2, 1): [("t", "servo1_offset")],
        (2, 2): [("t", "servo2_offset")],

        (3, 0): [("t", "mx")],
        (3, 1): [("t", "my")],
        (3, 2): [("t", "mz")],

        (4, 0): [("t", "fx")],
        (4, 1): [("t", "fy")],
        (4, 2): [("t", "fz")],
    }

    fig_kalman, axe_kalman = plt.subplots(5, 3, figsize=(20, 10))
    set_plot_ranges(axe_kalman, plot_ranges, kalman_plot_indexes.items(), USE_LATEX)

    plot_history(kalman_state_history, kalman_plot_indexes, axe_kalman, r"Kalman state")

    axe_kalman[0][0].legend(loc='upper left')

plt.show()
