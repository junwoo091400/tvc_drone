#!/usr/bin/env python
import rospy

import numpy as np
import math
import serial
import pigpio 
import time

from drone_gnc.msg import DroneControl
from drone_gnc.msg import DroneState
from drone_gnc.msg import Sensor

def control_callback(control):
    top_motor_cmd = control.top
    bottom_motor_cmd = control.bottom

    # saturate inputs
    control.servo1 = min(max(control.servo1, -0.3), 0.3)
    control.servo2 = min(max(control.servo2, -0.3), 0.3)

    top_motor_cmd = min(max(top_motor_cmd, 0), 80)
    bottom_motor_cmd = min(max(bottom_motor_cmd, 0), 80)

    # convert to PWM
    servo1_DC = ((control.servo1/(np.pi/2)) + 1.453)*1000
    servo2_DC = ((-control.servo2/(np.pi/2)) + 1.47)*1000

    top_motor_DC = top_motor_cmd*10 + 1000
    bottom_motor_DC = bottom_motor_cmd*10 + 1000
    # print(top_motor_DC, bottom_motor_DC)
    # print(servo1_DC, servo2_DC)

    # send to motors
    pi.set_servo_pulsewidth(pitch_pin, servo2_DC)
    pi.set_servo_pulsewidth(yaw_pin, servo1_DC)

    pi.set_servo_pulsewidth(top_motor_pin, top_motor_DC)
    pi.set_servo_pulsewidth(bottom_motor_pin, bottom_motor_DC)

if __name__ == '__main__':
    # Create global variable
    rocket_state = DroneState()

    sensor_data = Sensor()

    # Init ROS
    rospy.init_node('avionic_bridge', anonymous=True)
    
    # Subscribe to rocket_control 
    rospy.Subscriber("/drone_control", DroneControl, control_callback)

    # Publisher for rocket state from AV control
    sensor_pub = rospy.Publisher('/sensors', Sensor, queue_size=10)
    
    yaw_pin = 23
    pitch_pin = 24
    top_motor_pin = 8
    bottom_motor_pin = 25

    pi = pigpio.pi()

    pi.set_servo_pulsewidth(top_motor_pin, 0)
    time.sleep(1)
    pi.set_servo_pulsewidth(top_motor_pin, 2000)
    time.sleep(1)
    pi.set_servo_pulsewidth(top_motor_pin, 1000)
    time.sleep(1)

    pi.set_servo_pulsewidth(bottom_motor_pin, 0)
    time.sleep(1)
    pi.set_servo_pulsewidth(bottom_motor_pin, 2000)
    time.sleep(1)
    pi.set_servo_pulsewidth(bottom_motor_pin, 1000)
    time.sleep(1)

    pi.set_servo_pulsewidth(yaw_pin, 1500)
    pi.set_servo_pulsewidth(pitch_pin, 1500)

    pi.set_servo_pulsewidth(top_motor_pin, 1000)
    pi.set_servo_pulsewidth(bottom_motor_pin, 1000)
    time.sleep(1)
 
    control = DroneControl()
    control.servo1 = 0
    control.servo2 = 0
    control.top = 0
    control.bottom = 0
    control_callback(control)
    
    # Config Rpi
    ser = serial.Serial('/dev/serial0', 115200)  # open serial port
    
    # Node rate in Hz
    rate = rospy.Rate(200)
    n_average = 100
    baro = np.zeros(n_average)

    while not rospy.is_shutdown():
    
        # Thread sleep time defined by rate
        rate.sleep()
        
        #-----------------------------------------------------------
        
        line = ser.readline()
        if line[0] == "S" and line[1] == "S" : # SS symbol = sensor message
            # Decode byte array
            line = line[2:-2]
            line_ascii = line.decode('ascii')
            line_ascii = line_ascii.split(',')
        
            rocket_state_raw = np.array(line_ascii, dtype = np.float)
            # print(rocket_state_raw)
            
            # Parse state and publish it on the /sensor_pub topic
            baro = np.append(baro[1:],rocket_state_raw[7]/100)
            new_baro = np.convolve(baro, np.ones(n_average)/n_average, mode='valid')
            
            sensor_data.IMU_acc.x = -rocket_state_raw[1]
            sensor_data.IMU_acc.y = -rocket_state_raw[2]
            sensor_data.IMU_acc.z = rocket_state_raw[3]

            sensor_data.IMU_gyro.x = -rocket_state_raw[4]
            sensor_data.IMU_gyro.y = -rocket_state_raw[5]
            sensor_data.IMU_gyro.z = rocket_state_raw[6]

            sensor_data.baro_height = new_baro
            
        sensor_pub.publish(sensor_data)