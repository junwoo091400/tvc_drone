#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import time

from drone_gnc.msg import DroneControl
from drone_gnc.msg import FSM
from drone_gnc.msg import DroneState
from drone_gnc.msg import Sensor

from drone_gnc.srv import GetFSM
from drone_gnc.srv import InterpolateControlSpline

# TODO
# LAUNCH_DELAY = 100
LAUNCH_DELAY = 0

if __name__ == '__main__':

    rospy.init_node('low_level_control', anonymous=True)

    rospy.wait_for_service('/interpolateControlSpline')
    client_control_interpolation = rospy.ServiceProxy('/interpolateControlSpline', InterpolateControlSpline, persistent=True)

    rospy.wait_for_service('/getFSM_gnc')
    client_fsm = rospy.ServiceProxy('/getFSM_gnc', GetFSM)

    drone_control_pub = rospy.Publisher('/drone_control', DroneControl, queue_size=10)

    control_law = DroneControl()

    i = 0

    # Node rate in Hz
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        resp = client_fsm()
        current_fsm = resp.fsm

        control_law.servo1 = 0
        control_law.servo2 = 0
        control_law.top = 0
        control_law.bottom = 0
        # print(seq_idx)
        if current_fsm.state_machine == "Idle":
            pass
        elif current_fsm.state_machine == "Launch":
            i +=1
            if i>LAUNCH_DELAY:
                control_law = client_control_interpolation(rospy.get_rostime().to_sec()).drone_control
        elif current_fsm.state_machine == "Coast":
            pass

        drone_control_pub.publish(control_law)

        # Thread sleep time defined by rate
        rate.sleep()