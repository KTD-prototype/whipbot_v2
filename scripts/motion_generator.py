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
from nav_msgs.msg import Odometry
from whipbot_v2.msg import PID_gains

# prepare global parameters for robot's state
g_current_robot_location = [0.0] * 3  # [x,y,theta]
g_current_robot_velocity = [0.0] * 2  # [linear, angular]

g_last_robot_location = [0.0] * 3  # [x,y,theta]
g_last_robot_velocity = [0.0] * 2  # [linear, angular]

g_target_robot_location = [0.0] * 3  # [x,y,theta]
g_target_robot_velocity = [0.0] * 2  # [linear, angular]

g_velocity_command = [0.0] * 2  # [linear, angular]

g_initial_target_angle = 45

# P & D for linear position, P for heading
g_gains_for_position_control = [0] * 3


def motion_generator():
    global g_current_robot_location, g_current_robot_velocity
    global g_target_robot_location, g_target_robot_velocity
    global g_velocity_command
    global g_initial_target_angle, g_gains_for_position_control

    if g_velocity_command[0] == 0 and g_velocity_command[1] == 0:
        target_angle = g_initial_target_angle + (
            g_current_robot_location[0] - g_target_robot_location[0])\
            * g_gains_for_position_control[0] + g_current_robot_velocity[0] * g_gains_for_position_control[1]
        target_rotation = (
            g_current_robot_location[2] - g_target_robot_location[2])\
            * g_gains_for_position_control[2]

    else:
        pass

    # restrict range
    # target angle from -1000 to 1000 [*0.001 rad]
    if target_angle > 1000:
        target_angle = 1000
    elif target_angle < -1000:
        target_angle = -1000
    # target_rotation from -1000 to 1000 [equal to pwm signal @12bit in MCU]
    if target_rotation > 1000:
        target_rotation = 1000
    elif target_rotation < -1000:
        target_rotation = -1000

    pub_target_angle.publish(target_angle)
    pub_target_rotation.publish(target_rotation)
    pass


def callback_update_PID_gains(new_PID_gains):
    g_gains_for_position_control = new_PID_gains.pid_gains_for_position_control
    # publish_current_gains()


def callback_update_odometry(wheel_odometry):
    global g_current_robot_location, g_current_robot_velocity

    # store robot's odometry information.
    # Be careful our parameters don't have equal dimensions to message /Odometry
    # store current velocity : [linear, angular]
    g_current_robot_velocity = (wheel_odometry.twist.twist.linear.x,
                                wheel_odometry.twist.twist.angular.z)
    # store current orientation :[x,y,z,w] and convert to euler angle
    current_robot_orientation = (wheel_odometry.pose.pose.orientation.x,
                                 wheel_odometry.pose.pose.orientation.y,
                                 wheel_odometry.pose.pose.orientation.z,
                                 wheel_odometry.pose.pose.orientation.w)
    current_robot_orientation_euler = tf.transformations.euler_from_quaternion(
        current_robot_orientation)
    # store current location::[x, y, theta]
    g_current_robot_location = (wheel_odometry.pose.pose.position.x,
                                wheel_odometry.pose.pose.position.y,
                                current_robot_orientation_euler[2])


# function to inform current PID gains
def publish_current_gains():
    global g_gains_for_position_control
    current_PID_gains = PID_gains()
    for i in range(3):
        current_PID_gains.pid_gains_for_posture.append(
            g_gains_for_position_control[i])
    pub_current_gains.publish(current_PID_gains)
    del current_PID_gains


if __name__ == '__main__':
    # initialize node as "mcu_interface"
    rospy.init_node('motion_generator')

    # publisher for robot's motion
    pub_target_angle = rospy.Publisher(
        'target_angle', Int16, queue_size=1, latch=True)
    pub_target_rotation = rospy.Publisher(
        'target_rotation', Int16, queue_size=1, latch=True)

    # publisher to inform current PID gains
    pub_current_gains = rospy.Publisher(
        'current_PID_gains', PID_gains, queue_size=1, latch=True)

    # (for tuning) subscriber for change PID gains via message
    rospy.Subscriber('new_PID_gains', PID_gains, callback_update_PID_gains)

    # subscriber to get wheel odometry data
    rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                     callback_update_odometry, queue_size=1)

    # set the loop rate at 50Hz (higher is better, but it looks 60Hz is MAXIMUM for my environment)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            motion_generator()

        except IOError:
            pass

        rate.sleep()
