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
from sensor_msgs.msg import Joy
from whipbot_v2.msg import PID_gains

# prepare global parameters for robot's state
# current motion of the robot
g_current_robot_location = [0.0] * 3  # [x,y,theta]
g_current_robot_velocity = [0.0] * 2  # [linear, angular]

# last motion of the robot
g_last_robot_location = [0.0] * 3  # [x,y,theta]
g_last_robot_velocity = [0.0] * 2  # [linear, angular]

# desired motion of the robot
g_target_robot_location = [0.0] * 3  # [x,y,theta]
g_target_robot_velocity = [0.0] * 2  # [linear, angular]

# velocity command sent from other nodes
# command from joystick
g_velocity_command_joy = [0.0] * 2  # [linear, angular]
# command from autonomous driving node
g_velocity_command_autonomous = [0.0] * 2

# gains sent from GUI
g_gains_for_linear_velocity = [0] * 3  # P,I,D gain
g_gains_for_angular_velocity = [0] * 3  # P,I,D gain
# P & D gain for linear position, P gain for heading
g_gains_for_position_control = [0] * 3

g_last_time = 0  # timestamp to calculate acceleration of the robot


# fixed parameters
# default target tilt angle of the robot
g_initial_target_angle = 40

# gain for velocity command from joystick
JOY_GAIN_LINEAR = 0.5
JOY_GAIN_ANGULAR = 0.5


def motion_generator():
    # declare global parameters
    global g_current_robot_location, g_current_robot_velocity
    global g_target_robot_location, g_target_robot_velocity
    global g_velocity_command_joy, g_velocity_command_autonomous
    global g_initial_target_angle, g_last_time
    global g_gains_for_position_control
    global g_gains_for_linear_velocity, g_gains_for_angular_velocity

    # calculate acceleration of the robot
    # calculate delta t from last loop
    current_time = time.time()
    delta_t = current_time - g_last_time
    g_last_time = current_time
    # calculate acceleration
    robot_linear_accel = (
        g_current_robot_velocity[0] - g_last_robot_velocity[0]) / delta_t
    robot_angular_accel = (
        g_current_robot_velocity[1] - g_last_robot_velocity[1]) / delta_t

    # if there're velocity command from joy, control robot's motion by-
    # it's velocity at 1st priority (enable override on autonomous drive command)
    print(g_velocity_command_joy)
    if g_velocity_command_joy[0] != 0 or g_velocity_command_joy[1] != 0:
        # calculate target tilt angle of the robot based on it's velocity
        target_angle = g_initial_target_angle + \
            (-1) * (g_velocity_command_joy[0] - g_current_robot_velocity[0]) * \
            g_gains_for_linear_velocity[0] + \
            robot_linear_accel * g_gains_for_linear_velocity[2]
        # calculate rotation command for the robot based on it's velocity
        # be careful it looks like velocity feedback control, but "target_rotation"
        # doesn't mean angular velocity. It means bias for motor command between L/R to change robot's heading
        target_rotation = (g_velocity_command_joy[1] - g_current_robot_velocity[1]) * \
            (-1) * g_gains_for_angular_velocity[0] - \
            robot_angular_accel * g_gains_for_angular_velocity[2]

    # otherwise, control based on autonomous drive mode
    elif g_velocity_command_autonomous[0] != 0 or g_velocity_command_autonomous[1] != 0:
        # autonomous maneuver mode are not implemented yet
        pass

    # if there're no velocity command, control the robot to maintain current location and heading
    else:
        # calculate robot's desired angle and rotation based on the error of it's location, and it's heading
        target_angle = g_initial_target_angle + (
            g_current_robot_location[0] - g_target_robot_location[0])\
            * g_gains_for_position_control[0] + g_current_robot_velocity[0] * g_gains_for_position_control[1]
        target_rotation = (
            g_current_robot_location[2] - g_target_robot_location[2])\
            * g_gains_for_position_control[2]

    # restrict range of motion command
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

    # publish motion command as messages
    pub_target_angle.publish(target_angle)
    pub_target_rotation.publish(target_rotation)


def callback_update_PID_gains(new_PID_gains):
    global g_gains_for_linear_velocity, g_gains_for_angular_velocity
    global g_gains_for_position_control

    # contain new gains to parameters
    # contain gains for posture control only for display it
    gains_for_posture_control = new_PID_gains.pid_gains_for_posture
    # contain the other gains for motion command
    g_gains_for_linear_velocity = new_PID_gains.pid_gains_for_linear_velocity
    g_gains_for_angular_velocity = new_PID_gains.pid_gains_for_angular_velocity
    g_gains_for_position_control = new_PID_gains.pid_gains_for_position_control

    # display current gains
    rospy.loginfo("posture control gains : " + gains_for_posture_control)
    rospy.loginfo("linear velo ctrl gains ; " + g_gains_for_linear_velocity)
    rospy.loginfo("angular velo ctrl gains : " + g_gains_for_angular_velocity)
    rospy.loginfo("position control gains : " + g_gains_for_position_control)
    # publish_current_gains()
    # print(g_gains_for_position_control)


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


def callback_update_joycommand(joy_msg):
    global g_velocity_command_joy
    global JOY_GAIN_LINEAR, JOY_GAIN_ANGULAR

    # get command from left joy stick as velocity commands
    g_velocity_command_joy[0] = joy_msg.axes[1] * \
        JOY_GAIN_LINEAR  # [m/s] left stick F/R
    g_velocity_command_joy[1] = joy_msg.axes[0] * \
        JOY_GAIN_ANGULAR  # left stick L/R


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

    # subscriber for joystick command
    rospy.Subscriber('joy', Joy, callback_update_joycommand, queue_size=1)

    # set the loop rate at 50Hz (higher is better, but it looks 60Hz is MAXIMUM for my environment)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            motion_generator()

        except IOError:
            pass

        rate.sleep()
