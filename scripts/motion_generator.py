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
g_velocity_command_flag = False  # if False, disable remote control
# command from autonomous driving node
g_velocity_command_autonomous = [0.0] * 2

# gains sent from GUI
g_gains_for_linear_velocity = [0] * 3  # P,I,D gain
g_gains_for_angular_velocity = [0] * 3  # P,I,D gain
# P & D gain for linear position, P gain for heading
g_gains_for_position_control = [0] * 3

g_last_time = 0  # timestamp to calculate acceleration of the robot


# target tilt angle of the robot
g_initial_target_angle = 45
g_target_angle = 0
g_last_target_angle = 0

# global parameters for calibrating initial target angle
# timestamp for calibration
g_start_time_calib = 0.0
g_passed_time_calib = 0.0

# if True, execute calibration of the initial target angle
g_start_calibration_flag = False
# whether it is a first loop of calibration process
g_first_loop_of_calibration = True
# parameters for Max and min of ocsilation of location during hovering
g_maximum_locatin_during_hover = 0.0
g_minimum_location_during_hover = 0.0
# gain to calibrate target angle
CALIBRATION_GAIN = 30

# gain for velocity command from joystick
JOY_GAIN_LINEAR = 0.1
JOY_GAIN_ANGULAR = 1


def motion_generator():
    # declare global parameters
    global g_current_robot_location, g_current_robot_velocity
    global g_target_robot_location, g_target_robot_velocity
    global g_velocity_command_joy, g_velocity_command_flag
    global g_velocity_command_autonomous
    global g_initial_target_angle, g_target_angle, g_last_target_angle, g_last_time
    global g_gains_for_position_control
    global g_gains_for_linear_velocity, g_gains_for_angular_velocity

    # flag if there are commands
    linear_command_flag = False
    angular_command_flag = False

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

    # process to control robot's target angle based on target of robot location
    g_target_angle = g_initial_target_angle + \
        (g_current_robot_location[0] - g_target_robot_location[0]) * \
        g_gains_for_position_control[0] + \
        g_current_robot_velocity[0] * g_gains_for_position_control[1]

    # process to stabilize robot's heading (can be overrided by joystick)
    # if heading value are oscillating between -pi and pi, then do nothing
    if g_current_robot_location[2] * g_target_robot_location[2] < -4.0:
        pwm_offset_rotation = 0
    else:
        pwm_offset_rotation = (g_current_robot_location[2] -
                               g_target_robot_location[2]) * g_gains_for_position_control[2]

    # if there're velocity command from joy, control robot's motion by -
    # it's velocity at 1st priority (enable override on autonomous drive command)
    # first of all, check the flag for remote control
    if g_velocity_command_flag == True:
        if g_velocity_command_joy[0] != 0:
            linear_command_flag = True
            # calculate target tilt angle of the robot based on it's velocity
            g_target_robot_location[0] = g_target_robot_location[0] + g_velocity_command_joy[0] * \
                g_gains_for_linear_velocity[0] * 0.001 + robot_linear_accel * g_gains_for_linear_velocity[2] * 0.001

        # calculate rotation command for the robot based on it's velocity
        # be careful it looks like velocity feedback control, but "pwm_offset_rotation"
        # doesn't mean angular velocity. It means bias for motor command between L/R to change robot's heading
        if g_velocity_command_joy[1] != 0:
            angular_command_flag = True
            # calculate target rotation power of the robot
            pwm_offset_rotation = pwm_offset_rotation + (g_velocity_command_joy[1] - g_current_robot_velocity[1]) * \
                (-1) * g_gains_for_angular_velocity[0] - \
                robot_angular_accel * g_gains_for_angular_velocity[2]
            g_target_robot_location[2] = g_current_robot_location[2]

    # otherwise, control based on autonomous drive mode
    if g_velocity_command_autonomous[0] != 0 or g_velocity_command_autonomous[1] != 0:
        # autonomous maneuver mode are not implemented yet
        pass

    # if velocity command has stoped, refresh target location by current location
    if linear_command_flag == True and g_velocity_command_joy[0] == 0:
        g_target_robot_location[0] = g_current_robot_location[0]
        g_target_robot_location[1] = g_current_robot_location[1]
        print("stopped teleop")
        linear_command_flag = False
    if angular_command_flag == True and g_velocity_command_joy[1] == 0:
        g_target_robot_location[2] = g_current_robot_location[2]
        print("stopped teleop")
        angular_command_flag = False

    # ramp target_angle
    g_target_angle = ramp_target_angle(g_target_angle, g_last_target_angle)

    # restrict range of motion command
    # target angle from -1000 to 1000 [*0.001 rad]
    if g_target_angle > 1000:
        g_target_angle = 1000
    elif g_target_angle < -1000:
        g_target_angle = -1000
    # pwm_offset_rotation from -1000 to 1000 [equal to pwm signal @12bit in MCU]
    if pwm_offset_rotation > 1000:
        pwm_offset_rotation = 1000
    elif pwm_offset_rotation < -1000:
        pwm_offset_rotation = -1000

    # publish motion command as messages
    pub_target_angle.publish(g_target_angle)
    pub_pwm_offset_rotation.publish(pwm_offset_rotation)
    # print(pwm_offset_rotation)
    print(g_target_robot_location)
    # print(robot_linear_accel)
    # print(g_target_angle, pwm_offset_rotation)
    # print("")
    g_last_target_angle = g_target_angle


def ramp_target_angle(target, last_target):
    RAMP_FACTOR = 80
    if target > last_target + RAMP_FACTOR:
        target = last_target + RAMP_FACTOR
        print("ramped")
    elif target < last_target - RAMP_FACTOR:
        target = last_target - RAMP_FACTOR
        print("ramped")
    return target


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
    rospy.loginfo("posture control gains : " + str(gains_for_posture_control))
    rospy.loginfo("linear velo ctrl gains ; " +
                  str(g_gains_for_linear_velocity))
    rospy.loginfo("angular velo ctrl gains : " +
                  str(g_gains_for_angular_velocity))
    rospy.loginfo("position control gains : " +
                  str(g_gains_for_position_control))


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
    global g_velocity_command_joy, g_velocity_command_flag
    global JOY_GAIN_LINEAR, JOY_GAIN_ANGULAR
    global g_start_calibration_flag

    # get trigger for velocity command
    g_velocity_command_flag = joy_msg.buttons[4]
    # get command from left joy stick as velocity commands
    g_velocity_command_joy[0] = joy_msg.axes[1] * \
        JOY_GAIN_LINEAR  # [m/s] left stick F/R

    g_velocity_command_joy[1] = joy_msg.axes[0] * \
        JOY_GAIN_ANGULAR  # left stick L/R

    # check calibration trigger only if the calibration flag is FALSE
    if g_start_calibration_flag == False:
        # get tregger for calibrating initial target angle
        g_start_calibration_flag = joy_msg.buttons[5]

    # if the calibration flag is true, run calibration process
    if g_start_calibration_flag == True:
        calibrate_initial_target_angle()


# calibration process of target angle
def calibrate_initial_target_angle():
    global g_start_time_calib
    global g_start_calibration_flag, g_first_loop_of_calibration
    global g_maximum_locatin_during_hover, g_minimum_location_during_hover
    global g_current_robot_location, g_initial_target_angle
    global CALIBRATION_GAIN

    # if it is a first loop of calibration process, update parameters for calibration
    if g_first_loop_of_calibration == True:
        g_start_time_calib = time.time()
        g_maximum_locatin_during_hover = g_current_robot_location[0]
        g_minimum_location_during_hover = g_current_robot_location[0]
        g_first_loop_of_calibration = False
        rospy.loginfo("calibration started at " + str(g_start_time_calib))

    # update Max and min of the oscillation of the location during hovering
    if g_current_robot_location[0] > g_maximum_locatin_during_hover:
        g_maximum_locatin_during_hover = g_current_robot_location[0]
    if g_current_robot_location[0] < g_minimum_location_during_hover:
        g_minimum_location_during_hover = g_current_robot_location[0]

    # if 6 seconds have passed since the process started, calibrate target angle
    if time.time() - g_start_time_calib > 6:
        mean_location_during_hovering = (
            g_maximum_locatin_during_hover + g_minimum_location_during_hover) / 2 - g_target_robot_location[0]

        # update initial target angle
        g_initial_target_angle = g_initial_target_angle + \
            mean_location_during_hovering * CALIBRATION_GAIN

        # reset every flags and parameters after calibration
        g_start_calibration_flag = False
        g_first_loop_of_calibration = True

        # inform calibration has ended
        rospy.loginfo("calibration completed! target_angle : " +
                      str(g_initial_target_angle) + " [* 0.001 rad]")


# function to inform current PID gains
# def publish_current_gains():
#     global g_gains_for_position_control
#     current_PID_gains = PID_gains()
#     for i in range(3):
#         current_PID_gains.pid_gains_for_posture.append(
#             g_gains_for_position_control[i])
#     pub_current_gains.publish(current_PID_gains)
#     del current_PID_gains

if __name__ == '__main__':
    # initialize node as "mcu_interface"
    rospy.init_node('motion_generator')

    # publisher for robot's motion
    pub_target_angle = rospy.Publisher(
        'target_angle', Int16, queue_size=1, latch=True)
    pub_pwm_offset_rotation = rospy.Publisher(
        'pwm_offset_rotation', Int16, queue_size=1, latch=True)

    # publisher to inform current PID gains
    pub_current_gains = rospy.Publisher(
        'current_PID_gains', PID_gains, queue_size=1, latch=True)

    # (for tuning) subscriber for change PID gains via message
    rospy.Subscriber('new_PID_gains', PID_gains, callback_update_PID_gains)

    # subscriber to get wheel odometry data
    rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                     callback_update_odometry, queue_size=1)

    # subscriber for joystick command
    rospy.Subscriber('joy', Joy, callback_update_joycommand,
                     queue_size=1)

    # set the loop rate at 50Hz (higher is better, but it looks 60Hz is MAXIMUM for my environment)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            motion_generator()

        except IOError:
            pass

        rate.sleep()
