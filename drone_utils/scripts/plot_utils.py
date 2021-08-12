# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

def convert_state_to_array(state):
    q = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
    if np.isnan(q).any():
        q = [0, 0, 0, 1]
    r = R.from_quat(q)
    attitude_euler = r.as_euler('xyz', degrees=True)
    message_type = str(state._type)
    state_array = None
    if message_type == 'geometry_msgs/PoseStamped':
        state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                                0, 0, 0,
                                attitude_euler[0], attitude_euler[1], attitude_euler[2],
                                0, 0, 0,
                                0,
                                0,
                                0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                ])
    elif message_type == 'drone_gnc/DroneState':
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
    else:
        print('type error')

    return state_array


def convert_control_to_array(control):
    control_array = np.array([control.servo1, control.servo2, control.bottom, control.top])
    return control_array

def read_state_history(bag, topic, t_init, t_end):
    state_history = np.empty((0, NX + NP + 1))
    for _, msg, _ in bag.read_messages(topics=[topic]):
        t = msg.header.stamp.to_sec()
        if t > t_init and t < t_end:
            state_array = np.append(t - t_init, convert_state_to_array(msg))
            state_history = np.vstack((state_history, state_array))
    return state_history

def read_control_history(bag, topic, t_init, t_end):
    control_history = np.empty((0, NU + 1))
    for _, msg, _ in bag.read_messages(topics=[topic]):
        t = msg.header.stamp.to_sec()
        if t > t_init and t < t_end:
            control_array = np.append(t - t_init, convert_control_to_array(msg))
            control_history = np.vstack((control_history, control_array))
    return control_history

def read_horizon_history(bag, topic, t_init, t_end):
    NNODE = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        NNODE = len(msg.trajectory)
        break

    state_horizon_history = np.empty((NNODE, NX + NP + 1, 0))
    control_horizon_history = np.empty((NNODE, NU + 1, 0))
    for _, msg, _ in bag.read_messages(topics=[topic]):
        t = msg.header.stamp.to_sec()
        if t > t_init and t < t_end:
            state_horizon = np.empty((0, NX + NP + 1))
            control_horizon = np.empty((0, NU + 1))
            for waypoint_stamped in msg.trajectory:
                x = waypoint_stamped.state.pose.orientation.x
                y = waypoint_stamped.state.pose.orientation.y
                z = waypoint_stamped.state.pose.orientation.z
                w = waypoint_stamped.state.pose.orientation.w

                state_array = np.concatenate((
                    np.array([waypoint_stamped.header.stamp.to_sec() - t_init]),
                    convert_state_to_array(waypoint_stamped.state)
                ))
                control_array = np.concatenate((
                    np.array([waypoint_stamped.header.stamp.to_sec() - t_init]),
                    convert_control_to_array(waypoint_stamped.control)
                ))
                state_horizon = np.vstack((state_horizon, state_array))
                control_horizon = np.vstack((control_horizon, control_array))
            state_horizon_history = np.dstack((state_horizon_history, state_horizon))
            control_horizon_history = np.dstack((control_horizon_history, control_horizon))
    return state_horizon_history, control_horizon_history

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

    for plot_idx, name_list in plot_indexes.items():
        for (x_name, y_name) in name_list:
            x_data = history[:, var_indexes[x_name]]
            y_data = history[:, var_indexes[y_name]]
            line, = axe[plot_idx].plot(x_data, y_data, label=name, *plt_args)
            line_list.append((line, plot_idx))

    return line_list

def set_plot_ranges(axe, plot_ranges, plot_indexes):
    for plot_idx, name_list in plot_indexes:
        for (x_name, y_name) in name_list:
            axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                            xmax=plot_ranges[x_name][1],
                            ymin=plot_ranges[y_name][0],
                            ymax=plot_ranges[y_name][1])
            axe[plot_idx].set_xlabel(x_name)
            axe[plot_idx].set_ylabel(y_name)