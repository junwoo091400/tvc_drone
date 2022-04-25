#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# TODO use better plotting code from drone_utils instead

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import seaborn as sns

sns.set()
sns.set_style("whitegrid")

# sim_control = np.loadtxt(open("test_results/sim_control.csv", "rb"), delimiter=",")

try:
    sim_state = np.loadtxt(open("test_results/sim_state.csv", "rb"), delimiter=",")
    plt.plot(sim_state[0], sim_state[2], label="closed loop simulation")
except IOError:
    pass

try:
    guidance_traj = np.loadtxt(open("test_results/guidance_trajectory.csv", "rb"), delimiter=",")
    plt.plot(guidance_traj[0], guidance_traj[2], label="guidance trajectory")
except IOError:
    pass

plt.axis('scaled')
plt.xlabel("x [m]")
plt.ylabel("z [m]")
plt.legend()

plt.show()
