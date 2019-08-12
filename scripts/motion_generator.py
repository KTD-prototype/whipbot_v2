#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is for subscribe velocity command or other commands
# and generate target angle and rotation for actual robot's motion.
#


import rospy
import serial
import time
import signal
import sys
import tf
import math
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from whipbot_v2.msg import PID_gains
from wheel_odometry.msg import Encoder_2wheel

if __name__ == '__main__':
    # initialize node as "mcu_interface"
    rospy.init_node('mcu_interface')

    # publisher to inform current PID gains
    pub_current_gains = rospy.Publisher(
        'current_PID_gains', PID_gains, queue_size=1, latch=True)

    # publisher for encoder
    pub_encoders = rospy.Publisher(
        'encoder_2wheel', Encoder_2wheel, queue_size=1, latch=True)
    encoders = Encoder_2wheel()

    # define /Imu publisher and declare it
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
    imu_data = Imu()
    imu_data.header.frame_id = 'map'

    # (for tuning) subscriber for change PID gains via message
    rospy.Subscriber('new_PID_gains', PID_gains, callback_update_PID_gains)

    # subscriber to get target angle and target rotation of the robot
    rospy.Subscriber('target_angle', Int16,
                     callback_refresh_target_angle, queue_size=1)
    rospy.Subscriber('target_rotation', Int16,
                     callback_refresh_target_angle, queue_size=1)

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
