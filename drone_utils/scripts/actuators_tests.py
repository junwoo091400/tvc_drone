#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import time

from drone_optimal_control.msg import DroneControl
from drone_optimal_control.msg import FSM
from drone_optimal_control.msg import DroneState
from drone_optimal_control.msg import Sensor
from std_msgs.msg import String
from drone_optimal_control.srv import GetFSM

# define the test sequence
PERIODIC_SEQUENCE = False
RESET_ROLL = False

# TIME_SEQUENCE = [2, 2]
SERVO1_SEQUENCE = [0]
SERVO2_SEQUENCE = [5]

TOP_SEQUENCE = []
BOTTOM_SEQUENCE = []
TOP_SEQUENCE = [0, 5]
BOTTOM_SEQUENCE = [0, 5]

TOP_SEQUENCE = [0, 0] + list(range(20, 96, 5))
BOTTOM_SEQUENCE = [0, 0] + list(range(20, 96, 5))
# TOP_SEQUENCE = [0, 0, 20, 90]
# BOTTOM_SEQUENCE = [0, 0, 20, 90]
TIME_SEQUENCE = (len(BOTTOM_SEQUENCE)) * [2]

# AVERAGE_SEQ = range(30, 81, 5)

# AVERAGE_SEQ_REP = sum([[i]*9 for i in AVERAGE_SEQ], [])

# DELTA_SEQ = sum([[-8, -6, -4, -2, 0, 2, 4, 6, 8] for i in AVERAGE_SEQ], [])

# TOP_SEQUENCE = [0] +[av+delta for av, delta in zip(AVERAGE_SEQ_REP, DELTA_SEQ)]
# BOTTOM_SEQUENCE = [0] + [av-delta for av, delta in zip(AVERAGE_SEQ_REP, DELTA_SEQ)]


# TIME_SEQUENCE = [5] + (len(TOP_SEQUENCE))*[2]


# TIME_SEQUENCE = [3, 1000]

# TOP_SEQUENCE = [20, 20]
# BOTTOM_SEQUENCE = [20, 20]
# TIME_SEQUENCE = [3, 3]

previous_state = DroneState()
current_state = DroneState()
roll_speed = 0
previous_roll_speed = 0


def stateCallback(state):
    global current_state, previous_state
    previous_state = current_state
    current_state = state


def sensorsCallback(sensor_data):
    global roll_speed, previous_roll_speed
    previous_roll_speed = roll_speed
    roll_speed = sensor_data.IMU_gyro.z


if __name__ == '__main__':

    current_control = DroneControl()

    # Init ROS
    rospy.init_node('actuators_test', anonymous=True)

    drone_control_pub = rospy.Publisher('/drone_control', DroneControl, queue_size=10)

    rospy.Subscriber("/drone_state", DroneState, stateCallback)

    rospy.Subscriber("/sensors", Sensor, sensorsCallback)

    info_pub = rospy.Publisher('/info_pub', String, queue_size=10)

    control_law = DroneControl()
    control_law.servo1 = 0
    control_law.servo2 = 0
    control_law.top = 0
    control_law.bottom = 0

    # Node rate in Hz
    rate = rospy.Rate(10)

    seq_time_start = None
    seq_idx = 0

    previous_t = 0

    info_pub.publish("starting")
    while not rospy.is_shutdown():
        # rospy.loginfo(current_fsm)
        control_law.servo1 = 0
        control_law.servo2 = 0
        control_law.top = 0
        control_law.bottom = 0
        pd_control = 0
        if seq_time_start is None:
            seq_time_start = rospy.get_rostime().to_sec()
        if rospy.get_rostime().to_sec() - seq_time_start > TIME_SEQUENCE[seq_idx]:
            if RESET_ROLL and abs(roll_speed) > 0.8:
                info_pub.publish("resetting_roll")
                kp = 4
                kd = 0
                # previous_roll_speed = previous_state.twist.angular.z
                dt = rospy.get_rostime().to_sec() - previous_t
                pd_control = -roll_speed * kp - (roll_speed - previous_roll_speed) / dt * kd

                pd_control = max(min(pd_control, 35), -35)
                rospy.loginfo(str(pd_control))
                print("acc_roll", (roll_speed - previous_roll_speed) / dt)
            else:
                info_pub.publish("starting")
                print(seq_idx)
                seq_idx += 1
                seq_time_start = rospy.get_rostime().to_sec()
                if seq_idx >= len(TIME_SEQUENCE):
                    if PERIODIC_SEQUENCE:
                        seq_idx = 0
                    else:
                        seq_idx = -10
                        coast_command = String("Coast")
                        continue
        control_law.servo1 = (SERVO1_SEQUENCE[seq_idx] if seq_idx < len(SERVO1_SEQUENCE) else SERVO1_SEQUENCE[
            0]) * np.pi / 180
        control_law.servo2 = (SERVO2_SEQUENCE[seq_idx] if seq_idx < len(SERVO2_SEQUENCE) else SERVO2_SEQUENCE[
            0]) * np.pi / 180
        control_law.top = (TOP_SEQUENCE[seq_idx] if seq_idx < len(TOP_SEQUENCE) else 0)
        control_law.bottom = (BOTTOM_SEQUENCE[seq_idx] if seq_idx < len(BOTTOM_SEQUENCE) else 0)
        print(control_law.bottom, control_law.top, control_law.servo1, control_law.servo2)

        # # TODO check orientation
        # if pd_control != 0:
        #     control_law.top = AVERAGE_SEQ_REP[seq_idx] - pd_control
        #     control_law.bottom = AVERAGE_SEQ_REP[seq_idx] + pd_control

        previous_t = rospy.get_rostime().to_sec()
        drone_control_pub.publish(control_law)
