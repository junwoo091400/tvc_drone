# -*- coding: utf-8 -*-

import rosbag
import time
import os
from shutil import move
import numpy as np
from .constants import var_titles_latex, var_titles
import matplotlib.pyplot as plt


def plot_df(df, x_name, y_name, axe=None, scaled_xy=False, use_latex=False, plot_ranges=None, **plt_args):
    if axe is None:
        fig, axe = plt.subplots()

    axe = np.array(axe).flatten()[0]

    line, = axe.plot(df[x_name].to_numpy(), df[y_name].to_numpy(), **plt_args)

    if scaled_xy:
        axe.axis('scaled')

    if plot_ranges is not None:
        axe.axis(xmin=plot_ranges[x_name][0],
                 xmax=plot_ranges[x_name][1],
                 ymin=plot_ranges[y_name][0],
                 ymax=plot_ranges[y_name][1])

    if use_latex:
        axe.set_xlabel(var_titles_latex[x_name])
        axe.set_ylabel(var_titles_latex[y_name])
    else:
        axe.set_xlabel(var_titles.get(x_name, x_name))
        axe.set_ylabel(var_titles.get(y_name, y_name))

    return line


def plot_df_multiple(df, plot_indexes, axe, use_latex=False, plot_ranges=None, **plt_args):
    if len(axe.shape) == 1:
        axe = np.expand_dims(axe, 1)

    line_list = []

    for plot_idx, name_list in plot_indexes.items():
        for (x_name, y_name) in name_list:
            line = plot_df(df, x_name, y_name, axe[plot_idx], use_latex=use_latex, plot_ranges=plot_ranges, **plt_args)
            line_list.append((line, plot_idx))

    return line_list
