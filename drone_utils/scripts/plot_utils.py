# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from collections import OrderedDict
import rosbag
import time
import rospy
import os
from shutil import move


rad2deg = 180/np.pi

def convert_state_to_array(state):
    message_type = str(state._type)
    state_array = None
    if message_type == "geometry_msgs/PoseStamped":
        q = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
        if np.isnan(q).any():
            q = [0, 0, 0, 1]
        r = R.from_quat(q)
        attitude_euler = r.as_euler("xyz", degrees=True)
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
    elif message_type == "geometry_msgs/TwistStamped":
        state_array = np.array([0, 0, 0,
                                state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                                0, 0, 0,
                                state.twist.angular.x*rad2deg, state.twist.angular.y*rad2deg, state.twist.angular.z*rad2deg,
                                0,
                                0,
                                0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                ])
    elif message_type == "drone_gnc/DroneState":
        q = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
        if np.isnan(q).any():
            q = [0, 0, 0, 1]
        r = R.from_quat(q)
        attitude_euler = r.as_euler("xyz", degrees=True)
        state_array = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z,
                                state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                                attitude_euler[0], attitude_euler[1], attitude_euler[2],
                                state.twist.angular.x*rad2deg, state.twist.angular.y*rad2deg, state.twist.angular.z*rad2deg,
                                state.thrust_scaling,
                                state.torque_scaling,
                                state.servo1_offset*rad2deg, state.servo2_offset*rad2deg,
                                state.disturbance_force.x, state.disturbance_force.y, state.disturbance_force.z,
                                state.disturbance_torque.x, state.disturbance_torque.y, state.disturbance_torque.z,
                                ])
    else:
        print("type error")

    return state_array


def convert_control_to_array(control):
    control_array = np.array([control.servo1*rad2deg, control.servo2*rad2deg, control.bottom, control.top])
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
var_indexes = OrderedDict(OrderedDict([("t", 0)]).items() | state_indexes.items() | control_indexes.items() | param_indexes.items())


state_titles_latex = OrderedDict([
    ("x", r"$\boldsymbol{x}$ [m]"),
    ("y", r"$\boldsymbol{y}$ [m]"),
    ("z", r"$\boldsymbol{z}$ [m]"),
    ("dx", r"$\boldsymbol{v_x}$ [m/s]"),
    ("dy", r"$\boldsymbol{v_y}$ [m/s]"),
    ("dz", r"$\boldsymbol{v_z}$ [m/s]"),
    ("yaw (x)", r"$\boldsymbol{\alpha}$ [$\degree$]"),
    ("pitch (y)", r"$\boldsymbol{\beta}$ [$\degree$]"),
    ("roll (z)", r"$\boldsymbol{\gamma}$ [$\degree$]"),
    ("dyaw (x)", r"$\boldsymbol{\omega_x}$ [$\degree$/s]"),
    ("dpitch (y)", r"$\boldsymbol{\omega_y}$ [$\degree$/s]"),
    ("droll (z)", r"$\boldsymbol{\omega_z}$ [$\degree$/s]"),
])

control_titles_latex = OrderedDict([
    ("servo1", r"$\boldsymbol{\theta_1}$ [$\degree$]"),
    ("servo2", r"$\boldsymbol{\theta_2}$ [$\degree$]"),
    ("bottom", r"$\boldsymbol{P_B}$ [\%]"),
    ("top", r"$\boldsymbol{P_T}$ [\%]"),
])

param_titles_latex = OrderedDict([
    ("thrust_scaling", r"thrust scaling"),
    ("torque_scaling", r"torque scaling"),
    ("servo1_offset", r"servo1 offset [$\degree$]"),
    ("servo2_offset", r"servo2 offset [$\degree$]"),
    ("fx", r"$\boldsymbol{F_x}$ [N]"),
    ("fy", r"$\boldsymbol{F_y}$ [N]"),
    ("fz", r"$\boldsymbol{F_z}$ [N]"),
    ("mx", r"$\boldsymbol{M_x}$ [Nm]"),
    ("my", r"$\boldsymbol{M_y}$ [Nm]"),
    ("mz", r"$\boldsymbol{M_z}$ [Nm]"),
])
var_titles_latex = OrderedDict(OrderedDict([("t", r"$\boldsymbol{t}$ [s]")]).items() | state_titles_latex.items() | control_titles_latex.items() | param_titles_latex.items())


state_titles = OrderedDict([
    ("x", r"x [m]"),
    ("y", r"y [m]"),
    ("z", r"z [m]"),
    ("dx", r"v_x [m/s]"),
    ("dy", r"v_y [m/s]"),
    ("dz", r"v_z [m/s]"),
    ("yaw (x)", r"alpha [deg]"),
    ("pitch (y)", r"beta [deg]"),
    ("roll (z)", r"gamma [deg]"),
    ("dyaw (x)", r"w_x [deg/s]"),
    ("dpitch (y)", r"w_y[deg/s]"),
    ("droll (z)", r"w_z [deg/s]"),
])

control_titles = OrderedDict([
    ("servo1", r"servo1 [deg]"),
    ("servo2", r"servo2 [deg]"),
    ("bottom", r"P_B [\%]"),
    ("top", r"P_T [\%]"),
])

param_titles = OrderedDict([
    ("thrust_scaling", r"thrust scaling"),
    ("torque_scaling", r"torque scaling"),
    ("servo1_offset", r"servo1 offset [deg]"),
    ("servo2_offset", r"servo2 offset [deg]"),
    ("fx", r"F_x [N]"),
    ("fy", r"F_y [N]"),
    ("fz", r"F_z [N]"),
    ("mx", r"M_x [Nm]"),
    ("my", r"M_y [Nm]"),
    ("mz", r"M_z [Nm]"),
])
var_titles = OrderedDict(OrderedDict([("t", r"t [s]")]).items() | state_titles.items() | control_titles.items() | param_titles.items())


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

def set_plot_ranges(axe, plot_ranges, plot_indexes, use_latex=False):
    for plot_idx, name_list in plot_indexes:
        for (x_name, y_name) in name_list:
            axe[plot_idx].axis(xmin=plot_ranges[x_name][0],
                            xmax=plot_ranges[x_name][1],
                            ymin=plot_ranges[y_name][0],
                            ymax=plot_ranges[y_name][1])
            if use_latex:
                axe[plot_idx].set_xlabel(var_titles_latex[x_name])
                axe[plot_idx].set_ylabel(var_titles_latex[y_name])
            else:
                axe[plot_idx].set_xlabel(var_titles[x_name])
                axe[plot_idx].set_ylabel(var_titles[y_name])

def reorder_bag(bagfile):
    orig = os.path.splitext(bagfile)[0] + ".orig.bag"
    move(bagfile, orig)
    with rosbag.Bag(bagfile, 'w') as outbag:
        last_time = time.clock()
        buffer = []
        for topic, msg, t in rosbag.Bag(orig).read_messages():
            if msg._has_header:
                t = msg.header.stamp
            buffer.append((msg, topic, t))
            if len(buffer) > 50:
                msg_tuple = min(buffer, key=lambda m: m[2].to_sec())
                buffer.remove(msg_tuple)
                msg, topic, t = msg_tuple
                outbag.write(topic, msg, t)
    print("done reordering")