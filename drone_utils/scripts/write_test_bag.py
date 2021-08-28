#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import rosbag
import seaborn as sns
from drone_gnc.msg import DroneState
from numpy import sin, cos
import rospy

sns.set()

period = 0.005

import rosbag
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
rospack = rospkg.RosPack()
bag = rosbag.Bag(rospack.get_path('drone_utils') + '/log/sine_test.bag', 'w')
try:
    for t in np.arange(0.0001, 2, 0.04):
        state_msg = DroneState()
        state_msg.pose.position.x = sin(10*t)
        state_msg.twist.linear.x = 10*cos(10*t)
        state_msg.pose.orientation.w = 1
        state_msg.header.stamp = rospy.Time(t)
        bag.write('/simu_drone_state', state_msg, rospy.Time(t))
        bag.write('/drone_state', state_msg, rospy.Time(t))

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = -sin(10*t)
        pose_msg.pose.orientation.w = 1
        pose_msg.header.stamp = rospy.Time(t)
        bag.write('/optitrack_client/Kite/optitrack_pose', pose_msg)
finally:
    bag.close()