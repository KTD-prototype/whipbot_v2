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
from sensor_msgs.msg import Imu
from whipbot_v2.msg import PID_gains
from wheel_odometry.msg import Encoder_2wheel

# prepare serial port to communicate with the MCU
ser = serial.Serial('/dev/ESP32', 115200)

# global parameters for pid gains used in the MCU
global_pid_gain_posture = [0] * 3  # 3 gain parameters : P, I and D


def get_MCU_data():
    global global_pid_gain_posture

    command_head = 'H'

    target_angle = 80  # * 0.001 deg (mdeg)
    target_angular_velocity = 0

    target_angle_high = target_angle >> 8
    target_angle_low = target_angle & 0x00ff
    target_angular_velocity_high = target_angular_velocity >> 8
    target_angular_velocity_low = target_angular_velocity & 0x00ff

    P_gain_posture_high = global_pid_gain_posture[0] >> 8
    P_gain_posture_low = global_pid_gain_posture[0] & 0x00ff
    I_gain_posture_high = global_pid_gain_posture[1] >> 8
    I_gain_posture_low = global_pid_gain_posture[1] & 0x00ff
    D_gain_posture_high = global_pid_gain_posture[2] >> 8
    D_gain_posture_low = global_pid_gain_posture[2] & 0x00ff

    send_command = []
    send_command += [command_head, chr(target_angle_high), chr(
        target_angle_low), chr(target_angular_velocity_high), chr(
        target_angular_velocity_low), chr(P_gain_posture_high), chr(
        P_gain_posture_low), chr(I_gain_posture_high), chr(
        I_gain_posture_low), chr(D_gain_posture_high), chr(
        D_gain_posture_low)]

    ser.reset_input_buffer()
    ser.write(send_command)
    # print(send_command)
    send_command = []

    # print(ser.inWaiting())
    while ser.inWaiting() < 60:
        # print(ser.inWaiting())
        pass
    # print('received')

    # i:0 to 15 : 16 data : encoder-L/R, roll, pitch, heading[rad],
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

        # store and publish encoder data
        encoders.left_encoder = data_from_MCU[0]
        encoders.right_encoder = data_from_MCU[1]
        pub_encoders.publish(encoders)

        # transform posture from euler to quaternion
        posture_angle_quaternion = tf.transformations.quaternion_from_euler(
            data_from_MCU[2], data_from_MCU[3], data_from_MCU[4])

        # contain to the message : /imu
        imu_data.orientation.x = posture_angle_quaternion[0]
        imu_data.orientation.y = posture_angle_quaternion[1]
        imu_data.orientation.z = posture_angle_quaternion[2]
        imu_data.orientation.w = posture_angle_quaternion[3]

        # contain data of accelerometer to the message : /imu
        imu_data.linear_acceleration.x = data_from_MCU[5]
        imu_data.linear_acceleration.y = data_from_MCU[6]
        imu_data.linear_acceleration.z = data_from_MCU[7]

        # contain data of gyroscope to the message : /imu
        imu_data.angular_velocity.x = data_from_MCU[8]
        imu_data.angular_velocity.y = data_from_MCU[9]
        imu_data.angular_velocity.z = data_from_MCU[10]

        imu_pub.publish(imu_data)  # publish as sensor_msgs/Imu

        if data_from_MCU[15] > 11.1:  # 3.7V per cell @3s LiPo
            rospy.loginfo("battery voltage : " +
                          str(data_from_MCU[15]) + " [V]")
        elif data_from_MCU[15] > 10.8:  # 3.6V per cell @3s LiPo
            rospy.logwarn("battery voltage : " +
                          str(data_from_MCU[15]) + " [V]")
        elif data_from_MCU[15] > 10.2:  # 3.4V per cell @3s LiPo
            rospy.logfatal("battery voltage : " +
                           str(data_from_MCU[15]) + " [V]")


# callback function to refresh PID gains those are subscribed
def callback_update_PID_gains(new_PID_gains):
    global global_pid_gain_posture
    global_pid_gain_posture = new_PID_gains.pid_gains_for_posture

    # show new PID gains after refreshing
    display_current_gains()


# functions to display current PID gains to console
def display_current_gains():
    global global_pid_gain_posture
    rospy.loginfo("set PID gain for posture as " +
                  str(global_pid_gain_posture))
    print("")


# function to inform current PID gains
def publish_current_gains():
    global global_pid_gain_posture
    current_PID_gains = PID_gains()
    for i in range(3):
        current_PID_gains.pid_gains_for_posture.append(
            global_pid_gain_posture[i])
    pub_current_gains.publish(current_PID_gains)
    del current_PID_gains


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
