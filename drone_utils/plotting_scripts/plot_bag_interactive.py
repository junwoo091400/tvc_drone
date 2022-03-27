#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import re
from plot_utils import plot_df_multiple, parse_bag
import seaborn as sns
from matplotlib.widgets import Slider
import os
import rospkg
import numpy as np
import argparse


def plot_horizon_interactive(df, axe, plot_indexes, slider, **plt_args):
    # lambda function that returns the horizon at the given index
    get_horizon = lambda df, index, var_name: df.iloc[index].filter(regex="trajectory_\w_" + var_name + "$").to_numpy()

    line_list = []
    for plot_idx, name_list in plot_indexes.items():
        for (x_name, y_name) in name_list:
            line, = axe[plot_idx].plot(get_horizon(df, 0, x_name), get_horizon(df, 0, y_name), **plt_args)
            line_list.append((line, plot_idx))

    def update(_):
        time_val = slider.val
        time_array = df['t'].to_numpy()
        idx = min(np.searchsorted(time_array, time_val, side="left"), time_array.size - 1)

        for i in range(len(line_list)):
            l, plot_idx = line_list[i]
            for (x_name, y_name) in plot_indexes[plot_idx]:
                l.set_xdata(get_horizon(df, idx, x_name))
                l.set_ydata(get_horizon(df, idx, y_name))

        axe.figure.canvas.draw_idle()

    slider.on_changed(update)


rospack = rospkg.RosPack()

USE_LATEX = False
if USE_LATEX:
    plt.rc('font', family='serif', serif='cm10')
    plt.rc('text', usetex=True)
    plt.rc('text.latex', preamble=r'\usepackage{gensymb}\usepackage{amsmath}')

sns.set()
sns.set_style("whitegrid")

state_plot_indexes = {
    (0, 0): [("t", "x")],
    (0, 1): [("t", "y")],
    (0, 2): [("t", "z")],

    (1, 0): [("t", "vx")],
    (1, 1): [("t", "vy")],
    (1, 2): [("t", "vz")],

    (2, 0): [("t", "yaw")],
    (2, 1): [("t", "pitch")],
    (2, 2): [("t", "roll")],

    (3, 0): [("t", "wx")],
    (3, 1): [("t", "wy")],
    (3, 2): [("t", "wz")],
}

if __name__ == "__main__":
    default_bag_path = os.path.join(rospack.get_path('drone_utils'), 'log/log.bag')

    parser = argparse.ArgumentParser(description="Plot a ROS bag")
    parser.add_argument("rosbag_filepath", nargs='?', type=str, default=default_bag_path, help="rosbag file path")

    args = parser.parse_args()

    df_dict = parse_bag(args.rosbag_filepath, ["/drone_state", "/drone_control", "/control/debug/horizon"])

    # set initial times to 0
    t0 = min([df['t'][0] for df in df_dict.values()])
    for df in df_dict.values():
        df['t'] -= t0
    horizon_time_columns = [s for s in df_dict["/control/debug/horizon"].columns if
                            re.compile("trajectory_\w_t$").match(s)]
    df_dict["/control/debug/horizon"][horizon_time_columns] -= t0

    fig, axe = plt.subplots(5, 3, figsize=(15, 10))
    fig.subplots_adjust(wspace=0.4, hspace=0.5)

    plt.subplots_adjust(left=0.15, bottom=0.25)
    ax_slider = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='white')
    slider = Slider(ax_slider, 'Time', 0, df_dict["/drone_state"]['t'].iloc[-1], valinit=0)

    plot_df_multiple(df_dict["/drone_state"], state_plot_indexes, axe, label=r"Simulation state", use_latex=USE_LATEX)

    plot_horizon_interactive(df_dict["/control/debug/horizon"], axe, state_plot_indexes, slider, c='g')

    plt.show()
