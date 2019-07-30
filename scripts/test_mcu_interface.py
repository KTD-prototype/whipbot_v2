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
send_command = []
count = 0
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

now = 0.0
last = 0.0
passed_time = 0.0


def get_data():
    global send_command, now, last, passed_time

    command = 4000

    command_low = command & 127
    command_high = (command >> 7) & 127
    command_head = ((command >> 14) & 127) + 128
    send_command += [chr(command_head), chr(command_high), chr(command_low)]

    # command_high = command >> 8
    # command_low = command & 0x00ff
    # send_command += [chr(command_high), chr(command_low)]

    ser.reset_input_buffer()
    ser.write(send_command)

    while ser.inWaiting < 1:
        print("waiting")
        pass

    data = ser.readline()
    data = data.replace('\r\n', '')
    data = int(data)
    now = time.time()
    passed_time = now - last
    last = now
    print(data * 2)
    print(passed_time)
    print("")
    # time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('get_sernsor')

    # you should wait for a while until your arduino is ready
    time.sleep(1)
    print("started")

    # set the loop rate at 50Hz (higher is better, but it looks 60Hz is MAXIMUM for my environment)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            get_data()

        except IOError:
            pass

        rate.sleep()
