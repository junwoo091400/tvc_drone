#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from tvc_simulator.msg import FSM
from tvc_simulator.msg import DroneState
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor
from tvc_simulator.msg import Trajectory
from tvc_simulator.msg import Waypoint

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import time
import rosbag
import csv

rospack = rospkg.RosPack()
in_path = rospack.get_path('tvc_simulator') + '/postProcess/test_02_04_21_bags/log4_f.bag'
out_path = rospack.get_path('tvc_simulator') + '/postProcess/test_02_04_21_bags/log4_f_d.bag'

with rosbag.Bag(out_path, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(in_path).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/sensor_pub":
            outbag.write(topic, msg, t - rospy.Duration.from_sec(1.5))
        else:
            outbag.write(topic, msg, t)
