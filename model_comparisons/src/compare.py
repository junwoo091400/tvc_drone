#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import time
import matplotlib
import matplotlib.pyplot as plt

from drone_gnc.msg import DroneControl
from drone_gnc.msg import FSM
from drone_gnc.msg import DroneState
from drone_gnc.msg import Sensor
from drone_gnc.msg import Trajectory
from drone_gnc.msg import Waypoint

from std_msgs.msg import String
from drone_gnc.srv import GetFSM
from drone_gnc.srv import InterpolateControlSpline

previous_state = DroneState()
current_state = DroneState()


def convert_state_to_array(state):
    state_array = np.array([state.pose.position.x, state.pose.position.y,  state.pose.position.z,
                            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z,
                            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
                            state.twist.angular.x, state.twist.angular.y, state.twist.angular.z])
    return state_array

state_history = np.empty((0, 14))

horizon_array = None
def convert_horizon_to_array(horizon):
    global horizon_array
    # if horizon_array is None:
    horizon_array = np.empty((0, 4))
    for waypoint in horizon.trajectory:
        waypoint_array = np.array([waypoint.time, waypoint.position.x, waypoint.position.y, waypoint.position.z])
        horizon_array = np.vstack((horizon_array, waypoint_array))
    print(horizon_array)
    plt.cla()
    plt.plot(state_history[:, 0]-state_history[0, 0], state_history[:, 1], label="integrator")
    plt.plot(horizon_array[:, 0] + state_history[-1, 0]-state_history[0, 0], horizon_array[:, 1], label="mpc horizon")
    plt.draw()
    plt.pause(0.00000000001)



def stateCallback(state):
    global state_history
    state_array = np.append(rospy.get_time(), convert_state_to_array(state))
    state_history = np.vstack((state_history, state_array))



if __name__ == '__main__':

    current_control = DroneControl()

    # Init ROS
    rospy.init_node('plot_results', anonymous=True)

    # drone_control_pub = rospy.Publisher('drone_control', DroneControl, queue_size=10)

    rospy.Subscriber("drone_state", DroneState, stateCallback)

    # rospy.Subscriber("sensor_pub", Sensor, sensorsCallback)

    rospy.wait_for_service('getFSM_gnc')
    client_fsm = rospy.ServiceProxy('getFSM_gnc', GetFSM)

    rospy.wait_for_service('/interpolateControlSpline')
    client_control_interpolation = rospy.ServiceProxy('/interpolateControlSpline', InterpolateControlSpline)

    command_pub = rospy.Publisher('commands', String, queue_size=10)

    info_pub = rospy.Publisher('info_pub', String, queue_size=10)
    rospy.Subscriber("mpc_horizon", Trajectory, convert_horizon_to_array)

    plt.ion()
    plt.show()

    mpc_period = 0.03

    time.sleep(1)
    drone_control_list = []
    t_array = np.arange(0, 2, mpc_period)
    control_array = np.zeros((t_array.size, 4))

    for i in range(t_array.size):
        drone_control = client_control_interpolation(t_array[i]).drone_control
        drone_control_list.append(drone_control)
        control_array[i, :] = np.array([drone_control.servo1, drone_control.servo2, drone_control.bottom, drone_control.top])

    # print(control_array)

    t_start = rospy.get_time()


    time.sleep(1)
    command_pub.publish("Launch")
    i = 0
    # Node rate in Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # drone_control_pub.publish(drone_control_list[i])
        i += 1
        rate.sleep()

    # plt.ion()
    # plt.legend(loc="upper left")
    # plt.figure()
    # plt.plot(t_array, control_array[:, 3])

    # plt.show()
