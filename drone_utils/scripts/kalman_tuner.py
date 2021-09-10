#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag
import rospy

import seaborn as sns

sns.set()

from plot_utils import convert_state_to_array, convert_control_to_array, NP, NX, NU, set_plot_ranges, var_indexes,\
    plot_history, read_state_history, reorder_bag

rospack = rospkg.RosPack()
bag_name = rospack.get_path('drone_utils') +"/"+ rospy.get_param('/log_file')
bag = rosbag.Bag(bag_name)

time_init = 0
t_end = 0

for topic, msg, t in bag.read_messages(topics=['/drone_state']):
    if time_init == 0:
        time_init = msg.header.stamp.to_sec()
    t_end = msg.header.stamp.to_sec()

# reorder_bag(bag_name)


kalman_state_history = read_state_history(bag, '/drone_state', time_init, t_end)

# time_init = -20
# t_end = 1



plot_ranges = {
    "t": [0, t_end - time_init],
    "x": [-1, 1],
    "y": [-1, 1],
    "z": [-1, 3],
    "dx": [-1.5, 1.5],
    "dy": [-1.5, 1.5],
    "dz": [-1.5, 5],
    "yaw (x)": [-15, 15],
    "pitch (y)": [-15, 15],
    "roll (z)": [-90, 90],
    "dyaw (x)": [-50, 50],
    "dpitch (y)": [-50, 50],
    "droll (z)": [-25, 25],

    "servo1": [-15, 15],
    "servo2": [-15, 15],
    "bottom": [0, 100],
    "top": [0, 100],

    "thrust_scaling": [0, 1.5],
    "torque_scaling": [0, 1.5],
    "servo1_offset": [-15, 15],
    "servo2_offset": [-15, 15],
    "fx": [-5, 5],
    "fy": [-5, 5],
    "fz": [-5, 5],
    "mx": [-1, 1],
    "my": [-1, 1],
    "mz": [-1, 1],
}
kalman_plot_indexes = {
    (0, 0): [("t", "x")],
    (0, 1): [("t", "y")],
    (0, 2): [("t", "z")],
    (0, 3): [("t", "thrust_scaling")],

    (1, 0): [("t", "yaw (x)")],
    (1, 1): [("t", "pitch (y)")],
    (1, 2): [("t", "roll (z)")],
    (1, 3): [("t", "torque_scaling")],

    (2, 0): [("t", "dx")],
    (2, 1): [("t", "dy")],
    (2, 2): [("t", "dz")],
    (2, 3): [("t", "servo1_offset")],

    (3, 0): [("t", "dyaw (x)")],
    (3, 1): [("t", "dpitch (y)")],
    (3, 2): [("t", "droll (z)")],
    (3, 3): [("t", "servo2_offset")],

    (4, 0): [("t", "fx")],
    (4, 1): [("t", "fy")],
    (4, 2): [("t", "fz")],

    (5, 0): [("t", "mx")],
    (5, 1): [("t", "my")],
    (5, 2): [("t", "mz")],
}

Q_names = ['x', 'x', 'x',
           'dx', 'dx', 'dx',
           'att', 'att', 'att', 'att',
           'datt', 'datt', 'datt',
           'thrust_scaling',
           'torque_scaling',
           'disturbance_force', 'disturbance_force', 'disturbance_force_z',
           'disturbance_torque', 'disturbance_torque', 'disturbance_torque_z']

R_names = ['optitrack_x', 'optitrack_x', 'optitrack_x',
           'pixhawk', 'pixhawk', 'pixhawk', 'pixhawk',
           'pixhawk', 'pixhawk', 'pixhawk',
           'pixhawk', 'pixhawk', 'pixhawk']

fig_kalman, axe_kalman = plt.subplots(6, 4, figsize=(20, 10))
set_plot_ranges(axe_kalman, plot_ranges, kalman_plot_indexes.items())


# pixhawk_twist_history = read_state_history(bag, '/mavros/local_position/velocity_body', time_init, t_end)
# plot_history(pixhawk_twist_history, kalman_plot_indexes, axe_kalman, "")

try:
    from drone_gnc.srv import KalmanSimu
    from drone_gnc.msg import DroneTrajectory

    kalman_simu = rospy.ServiceProxy('/kalman_simu', KalmanSimu)
    kalman_line_list = plot_history(kalman_state_history, kalman_plot_indexes, axe_kalman, "kalman_state")

    # optitrack_state_history = read_state_history(bag, '/optitrack_client/Kite/optitrack_pose', time_init, t_end)
    # plot_history(optitrack_state_history, kalman_plot_indexes, axe_kalman, "optitrack")

    simu_state_history = read_state_history(bag, '/simu_drone_state', time_init, t_end)
    plot_history(simu_state_history, kalman_plot_indexes, axe_kalman, "simu")

    # plot "ground truth"
    # Q = [1, 1, 1, 2000, 2000, 2000, 1, 1, 1, 1, 4000, 4000, 4000] + [0 for i in range(NP)]
    # R = [0.001] * 13
    # resp = kalman_simu(Q, R, False)
    # ground_truth_state_history = np.empty((0, NX + NP + 1))
    # for waypoint in resp.trajectory.trajectory:
    #     state_array = np.append(waypoint.state.header.stamp.to_sec() - time_init, convert_state_to_array(waypoint.state))
    #     ground_truth_state_history = np.vstack((ground_truth_state_history, state_array))
    # plot_history(ground_truth_state_history, kalman_plot_indexes, axe_kalman, "ground_truth", 'r')

    # create sliders figure
    plt.figure(figsize=(9, 6))

    initial_Q = rospy.get_param('/navigation/predict_vars')
    initial_R = rospy.get_param('/navigation/update_vars')

    param_sliders = {}
    i = 0
    for var_name in set(Q_names): # convert to a set to get unique values
        ax_slider = plt.axes([0.25, 0.8 - i * 0.05, 0.65, 0.04], facecolor='white')
        param_sliders[var_name] = Slider(ax_slider, var_name, 0, max(initial_Q[var_name]*10, 1), valinit=initial_Q[var_name], valstep=0.0001)
        i += 1
    for var_name in set(R_names): # convert to a set to get unique values
        ax_slider = plt.axes([0.25, 0.8 - i * 0.05, 0.65, 0.04], facecolor='white')
        param_sliders[var_name] = Slider(ax_slider, var_name, 0, max(initial_R[var_name]*10, 1), valinit=initial_R[var_name], valstep=0.0001)
        i += 1

    def update_kalman(val):
        Q = [param_sliders[var_name].val for var_name in Q_names]
        R = [param_sliders[var_name].val for var_name in R_names]
        resp = kalman_simu(Q, R, True)

        new_kalman_state_history = np.empty((0, NX + NP + 1))
        for waypoint in resp.trajectory.trajectory:
            state_array = np.append(waypoint.state.header.stamp.to_sec() - time_init, convert_state_to_array(waypoint.state))
            new_kalman_state_history = np.vstack((new_kalman_state_history, state_array))

        for i in range(len(kalman_line_list)):
            l, plot_idx = kalman_line_list[i]
            for (x_name, y_name) in kalman_plot_indexes[plot_idx]:
                l.set_ydata(new_kalman_state_history[:, var_indexes[y_name]])
                l.set_xdata(new_kalman_state_history[:, var_indexes[x_name]])
        fig_kalman.canvas.draw_idle()


    for var_name, slider in param_sliders.items():
        slider.on_changed(update_kalman)

except rospy.ServiceException as e:
    print("c++ kalman_tuner node is not running")

plt.show()
