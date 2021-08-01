#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag
import rospy

import seaborn as sns

sns.set()

from plot_utils import convert_state_to_array, convert_control_to_array, NP, NX, NU, var_indexes, plot_history

rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/log1_f.bag')

time_init = 0
t_end = 0
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if time_init == 0:
        time_init = t.to_sec()
    t_end = t.to_sec()

kalman_state_history = np.empty((0, NX + NP + 1))
for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if t.to_sec() > time_init and t.to_sec() < t_end:
        state_array = np.append(t.to_sec() - time_init, convert_state_to_array(msg))
        kalman_state_history = np.vstack((kalman_state_history, state_array))

fig_kalman, axe_kalman = plt.subplots(5, 3, figsize=(20, 10))

plot_ranges = {
    "t": [0, 3],
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

Q_names = ['x', 'x', 'z',
           'dx', 'dx', 'dz',
           'att', 'att', 'att', 'att',
           'datt', 'datt', 'datt',
           'thrust_scaling',
           'torque_scaling',
           'servo_offset', 'servo_offset',
           'disturbance_force', 'disturbance_force', 'disturbance_force',
           'disturbance_torque', 'disturbance_torque', 'disturbance_torque']

# set plot ranges
for plot_idx, (x_name, y_name) in kalman_plot_indexes.items():
    axe_kalman[plot_idx].axis(xmin=plot_ranges[x_name][0],
                              xmax=plot_ranges[x_name][1],
                              ymin=plot_ranges[y_name][0],
                              ymax=plot_ranges[y_name][1])
    axe_kalman[plot_idx].set_xlabel(x_name)
    axe_kalman[plot_idx].set_ylabel(y_name)

try:
    from drone_gnc.srv import KalmanSimu
    from drone_gnc.msg import DroneTrajectory

    kalman_simu = rospy.ServiceProxy('/kalman_simu', KalmanSimu)
    kalman_line_list = plot_history(kalman_state_history, kalman_plot_indexes, axe_kalman, "kalman_state")

    plt.figure(figsize=(9, 6))

    initial_Q = rospy.get_param('/filter/predict_vars')

    param_sliders = {}
    i = 0
    for var_name in set(Q_names): # convert to a set to get unique values
        ax_slider = plt.axes([0.25, 0.8 - i * 0.05, 0.65, 0.04], facecolor='white')
        param_sliders[var_name] = Slider(ax_slider, var_name, 0, 10000, valinit=initial_Q[var_name], valstep=0.01)
        i += 1

    def update_kalman(val):
        Q = [param_sliders[var_name].val for var_name in Q_names]
        resp = kalman_simu(Q)

        new_kalman_state_history = np.empty((0, NX + NP + 1))
        t0 = resp.trajectory.trajectory[0].state.header.stamp.to_sec()
        for waypoint in resp.trajectory.trajectory:
            state_array = np.append(waypoint.state.header.stamp.to_sec() - t0, convert_state_to_array(waypoint.state))
            new_kalman_state_history = np.vstack((new_kalman_state_history, state_array))

        for i in range(len(kalman_line_list)):
            l, plot_idx = kalman_line_list[i]
            x_name, y_name = kalman_plot_indexes[plot_idx]
            l.set_ydata(new_kalman_state_history[:, var_indexes[y_name]])
            l.set_xdata(new_kalman_state_history[:, var_indexes[x_name]])
        fig_kalman.canvas.draw_idle()


    for var_name, slider in param_sliders.items():
        slider.on_changed(update_kalman)

except rospy.ServiceException as e:
    print("c++ kalman_tuner node is not running")

plt.show()
