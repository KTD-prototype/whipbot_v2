#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a interface node to arduino
# it collects IMU data from arduino.
#


import rospy
import serial
import time
import signal
import sys
import tf
import math
# from sensor_msgs.msg import Imu
# from whipbot.msg import Posture_angle

ser = serial.Serial('/dev/ESP32', 115200)
count = 0
command = 0

now = 0.0
last = 0.0
passed_time = 0.0

INITIALIZE_MESSAGE = 'ets Jun  8 2016 00:22:57'


def get_MCU_data():
    command_head = 'H'
    velocity_command_linear = 100
    velocity_command_angular = 100
    send_command = []
    send_command += [command_head,
                     chr(velocity_command_linear), chr(velocity_command_angular)]

    ser.reset_input_buffer()
    ser.write(send_command)
    # print(send_command)
    send_command = []

    # print(ser.inWaiting())
    while ser.inWaiting() < 60:
        # print(ser.inWaiting())
        pass
    # print('received')

    # i:0 to 14, encoder-L/R, roll, pitch, heading[rad],
    # accelX,Y,Z[m/s2], gyroX,Y,Z[rad/s], magX,Y,Z[uT], temperature[degC], battery_voltage[v]
    data_from_MCU = [0.0] * 16
    reset_flag = False
    for i in range(16):
        data_from_MCU[i] = ser.readline()  # read data line by line
        # remove return code and end point null
        data_from_MCU[i] = data_from_MCU[i].replace('\r\n', '')
        if data_from_MCU[i] == 'ets Jun  8 2016 00:22:57':
            time.sleep(5)
            ser.reset_input_buffer()
            reset_flag = True
            break

        # convert into float type parameter
        data_from_MCU[i] = float(data_from_MCU[i])

    if reset_flag == False:
        print(data_from_MCU)


if __name__ == '__main__':
    # initialize node as "mcu_interface"
    rospy.init_node('mcu_interface')

    # you should wait for a while until your arduino is ready
    time.sleep(5)
    print("started")

    # set the loop rate at 50Hz (higher is better, but it looks 60Hz is MAXIMUM for my environment)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            get_MCU_data()

        except IOError:
            pass

        rate.sleep()
