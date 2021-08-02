#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R


def convert_state_to_array(state):
    q = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
    if np.isnan(q).any():
        q = [0, 0, 0, 1]
    r = R.from_quat(q)
    attitude_euler = r.as_euler('xyz', degrees=True)
    state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            attitude_euler[0], attitude_euler[1], attitude_euler[2],
                            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z,
                            state.thrust_scaling,
                            state.torque_scaling,
                            state.servo1_offset, state.servo2_offset,
                            state.disturbance_force.x, state.disturbance_force.y, state.disturbance_force.z,
                            state.disturbance_torque.x, state.disturbance_torque.y, state.disturbance_torque.z,
                            ])
    return state_array


def convert_control_to_array(control):
    control_array = np.array([control.servo1, control.servo2, control.bottom, control.top])
    return control_array


NP = 10
NX = 12
NU = 4
from collections import OrderedDict

state_indexes = OrderedDict([
    ("x", 1),
    ("y", 2),
    ("z", 3),
    ("dx", 4),
    ("dy", 5),
    ("dz", 6),
    ("yaw (x)", 7),
    ("pitch (y)", 8),
    ("roll (z)", 9),
    ("dyaw (x)", 10),
    ("dpitch (y)", 11),
    ("droll (z)", 12)
])
control_indexes = OrderedDict([
    ("servo1", 1),
    ("servo2", 2),
    ("bottom", 3),
    ("top", 4),
])
param_indexes = OrderedDict([
    ("thrust_scaling", 13),
    ("torque_scaling", 14),
    ("servo1_offset", 15),
    ("servo2_offset", 16),
    ("fx", 17),
    ("fy", 18),
    ("fz", 19),
    ("mx", 20),
    ("my", 21),
    ("mz", 22),
])
var_indexes = OrderedDict(OrderedDict([("t", 0)]).items() + state_indexes.items() + control_indexes.items() + param_indexes.items())


def plot_history(history, plot_indexes, axe, name, *plt_args):
    if history.shape[0] == 0:
        return
    line_list = []

    for plot_idx, (x_name, y_name) in plot_indexes.items():
        x_data = history[:, var_indexes[x_name]]
        y_data = history[:, var_indexes[y_name]]
        line, = axe[plot_idx].plot(x_data, y_data, label=name, *plt_args)
        line_list.append((line, plot_idx))

    return line_list
