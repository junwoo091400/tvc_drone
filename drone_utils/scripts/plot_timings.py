#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag
import seaborn as sns
import rospy

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
