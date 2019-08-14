#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is for setting parameters for PID control.
#


import rospy
import serial
import time
import signal
import sys
import tf
import math
from Tkinter import *
from whipbot_v2.msg import PID_gains

current_PID_gains_posture = [0] * 3  # P, I, D
current_PID_gains_linear_velocity = [0] * 3  # P, I, D
current_PID_gains_angular_velocity = [0] * 3  # P, I, D
# P & D for linear position, P for heading
current_PID_gains_positon_control = [0] * 3

DEFAULT_PID_GAINS_POSTURE = [1750, 0, 4]  # P, I, D
DEFAULT_PID_GAINS_LINEAR_VELOCITY = [0] * 3  # P, I, D
DEFAULT_PID_GAINS_ANGULAR_VELOCITY = [0] * 3  # P, I, D
# P & D for linear position, P for heading
DEFAULT_PID_GAINS_POSITION_CONTROL = [125, 360, 130]

new_PID_gains_posture = [0] * 3  # P, I, D
new_PID_gains_linear_velocity = [0] * 3  # P, I, D
new_PID_gains_angular_velocity = [0] * 3  # P, I, D
# P & D for linear position, P for heading
new_PID_gains_position_control = [0] * 3


class Test(Frame):
    def publish_new_gains(self):
        new_PID_gains = PID_gains()
        for i in range(3):
            new_PID_gains.pid_gains_for_posture.append(
                new_PID_gains_posture[i])
            new_PID_gains.pid_gains_for_linear_velocity.append(
                new_PID_gains_linear_velocity[i])
            new_PID_gains.pid_gains_for_angular_velocity.append(
                new_PID_gains_angular_velocity[i])
            new_PID_gains.pid_gains_for_position_control.append(
                new_PID_gains_position_control[i])
        pub_new_gains.publish(new_PID_gains)
        del new_PID_gains

    def set_Pgain_for_posture(self, val):
        global new_PID_gains_posture
        # print "slider now at", val
        val = int(val)
        new_PID_gains_posture = list(new_PID_gains_posture)
        new_PID_gains_posture[0] = val
        self.publish_new_gains()

    def set_Igain_for_posture(self, val):
        global new_PID_gains_posture
        # print "slider now at", val
        val = int(val)
        new_PID_gains_posture = list(new_PID_gains_posture)
        new_PID_gains_posture[1] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Dgain_for_posture(self, val):
        global new_PID_gains_posture
        # print "slider now at", val
        val = int(val)
        new_PID_gains_posture = list(new_PID_gains_posture)
        new_PID_gains_posture[2] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Pgain_for_linear_velocity(self, val):
        global new_PID_gains_linear_velocity
        # print "slider now at", val
        val = int(val)
        new_PID_gains_linear_velocity = list(new_PID_gains_linear_velocity)
        new_PID_gains_linear_velocity[0] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Igain_for_linear_velocity(self, val):
        global new_PID_gains_linear_velocity
        # print "slider now at", val
        val = int(val)
        new_PID_gains_linear_velocity = list(new_PID_gains_linear_velocity)
        new_PID_gains_linear_velocity[1] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Dgain_for_linear_velocity(self, val):
        global new_PID_gains_linear_velocity
        # print "slider now at", val
        val = int(val)
        new_PID_gains_linear_velocity = list(new_PID_gains_linear_velocity)
        new_PID_gains_linear_velocity[2] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Pgain_for_angular_velocity(self, val):
        global new_PID_gains_angular_velocity
        # print "slider now at", val
        val = int(val)
        new_PID_gains_angular_velocity = list(new_PID_gains_angular_velocity)
        new_PID_gains_angular_velocity[0] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Igain_for_angular_velocity(self, val):
        global new_PID_gains_angular_velocity
        # print "slider now at", val
        val = int(val)
        new_PID_gains_angular_velocity = list(new_PID_gains_angular_velocity)
        new_PID_gains_angular_velocity[1] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Dgain_for_angular_velocity(self, val):
        global new_PID_gains_angular_velocity
        # print "slider now at", val
        val = int(val)
        new_PID_gains_angular_velocity = list(new_PID_gains_angular_velocity)
        new_PID_gains_angular_velocity[2] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Pgain_for_linear_position(self, val):
        global new_PID_gains_position_control
        # print "slider now at", val
        val = int(val)
        new_PID_gains_position_control = list(new_PID_gains_position_control)
        new_PID_gains_position_control[0] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Dgain_for_linear_position(self, val):
        global new_PID_gains_position_control
        # print "slider now at", val
        val = int(val)
        new_PID_gains_position_control = list(new_PID_gains_position_control)
        new_PID_gains_position_control[1] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def set_Pgain_for_heading(self, val):
        global new_PID_gains_position_control
        # print "slider now at", val
        val = int(val)
        new_PID_gains_position_control = list(new_PID_gains_position_control)
        new_PID_gains_position_control[2] = val
        new_PID_gains = PID_gains()
        self.publish_new_gains()

    def reset(self):
        # global DEFAULT_PID_GAINS_POSTURE, DEFAULT_PID_GAINS_LINEAR_VELOCITY
        self.slider_Pgain_pos.set(DEFAULT_PID_GAINS_POSTURE[0])
        self.slider_Igain_pos.set(DEFAULT_PID_GAINS_POSTURE[1])
        self.slider_Dgain_pos.set(DEFAULT_PID_GAINS_POSTURE[2])
        self.slider_Pgain_linVel.set(DEFAULT_PID_GAINS_LINEAR_VELOCITY[0])
        self.slider_Igain_linVel.set(DEFAULT_PID_GAINS_LINEAR_VELOCITY[1])
        self.slider_Dgain_linVel.set(DEFAULT_PID_GAINS_LINEAR_VELOCITY[2])
        self.slider_Pgain_angVel.set(DEFAULT_PID_GAINS_ANGULAR_VELOCITY[0])
        self.slider_Igain_angVel.set(DEFAULT_PID_GAINS_ANGULAR_VELOCITY[1])
        self.slider_Dgain_angVel.set(DEFAULT_PID_GAINS_ANGULAR_VELOCITY[2])
        self.slider_Pgain_position.set(DEFAULT_PID_GAINS_POSITION_CONTROL[0])
        self.slider_Dgain_position.set(DEFAULT_PID_GAINS_POSITION_CONTROL[1])
        self.slider_Pgain_heading.set(DEFAULT_PID_GAINS_POSITION_CONTROL[2])

    def createWidgets(self):
        # scale for posture control gains
        self.slider_Pgain_pos = Scale(self, from_=0, to=5000,
                                      # value=1800,
                                      variable=1800,
                                      orient=HORIZONTAL,
                                      length="8i",
                                      label="P gain for POS",
                                      command=self.set_Pgain_for_posture)
        # self.slider_Pgain_pos.set(current_PID_gains_posture[0])

        self.slider_Igain_pos = Scale(self, from_=0, to=500,
                                      # value=50,
                                      orient=HORIZONTAL,
                                      length="8i",
                                      label="I gain for POS",
                                      command=self.set_Igain_for_posture)
        # self.slider_Igain_pos.set(current_PID_gains_posture[1])

        self.slider_Dgain_pos = Scale(self, from_=0, to=50,
                                      # value=5,
                                      orient=HORIZONTAL,
                                      length="8i",
                                      label="D gain for POS",
                                      command=self.set_Dgain_for_posture)
        # self.slider_Dgain_pos.set(current_PID_gains_posture[2])

        # scale for linear velocity control gains
        self.slider_Pgain_linVel = Scale(self, from_=0, to=500,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="P gain for linVel",
                                         command=self.set_Pgain_for_linear_velocity)
        # self.slider_Pgain_linVel.set(current_PID_gains_linear_velocity[0])

        self.slider_Igain_linVel = Scale(self, from_=0, to=100,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="I gain for linVel",
                                         command=self.set_Igain_for_linear_velocity)
        # self.slider_Igain_linVel.set(current_PID_gains_linear_velocity[1])

        self.slider_Dgain_linVel = Scale(self, from_=0, to=100,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="D gain for linVel",
                                         command=self.set_Dgain_for_linear_velocity)
        # self.slider_Dgain_linVel.set(current_PID_gains_linear_velocity[2])

        # scale for angular velocity control gains
        self.slider_Pgain_angVel = Scale(self, from_=0, to=500,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="P gain for angVel",
                                         command=self.set_Pgain_for_angular_velocity)
        # self.slider_Pgain_angVel.set(current_PID_gains_angular_velocity[0])

        self.slider_Igain_angVel = Scale(self, from_=0, to=100,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="I gain for angVel",
                                         command=self.set_Igain_for_angular_velocity)
        # self.slider_Igain_angVel.set(current_PID_gains_angular_velocity[1])

        self.slider_Dgain_angVel = Scale(self, from_=0, to=500,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="D gain for angVel",
                                         command=self.set_Dgain_for_angular_velocity)
        # self.slider_Dgain_angVel.set(current_PID_gains_angular_velocity[2])

        # scale for position control gains
        self.slider_Pgain_position = Scale(self, from_=0, to=1000,
                                           # value=50,
                                           orient=HORIZONTAL,
                                           length="8i",
                                           label="P gain for position control",
                                           command=self.set_Pgain_for_linear_position)
        # self.slider_Pgain_position.set(current_PID_gains_positon_control[0])

        self.slider_Dgain_position = Scale(self, from_=0, to=1000,
                                           # value=50,
                                           orient=HORIZONTAL,
                                           length="8i",
                                           label="D gain for position control",
                                           command=self.set_Dgain_for_linear_position)
        # self.slider_Dgain_position.set(current_PID_gains_positon_control[1])

        self.slider_Pgain_heading = Scale(self, from_=0, to=1000,
                                          # value=50,
                                          orient=HORIZONTAL,
                                          length="8i",
                                          label="P gain for heading control",
                                          command=self.set_Pgain_for_heading)
        # self.slider_Pgain_heading.set(current_PID_gains_positon_control[2])

        self.reset = Button(self, text='reset slider',
                            command=self.reset)

        self.QUIT = Button(self, text='QUIT', foreground='red',
                           command=self.quit)

        self.slider_Pgain_pos.pack()
        self.slider_Igain_pos.pack()
        self.slider_Dgain_pos.pack()
        self.slider_Pgain_linVel.pack()
        self.slider_Igain_linVel.pack()
        self.slider_Dgain_linVel.pack()
        self.slider_Pgain_angVel.pack()
        self.slider_Igain_angVel.pack()
        self.slider_Dgain_angVel.pack()
        self.slider_Pgain_position.pack()
        self.slider_Dgain_position.pack()
        self.slider_Pgain_heading.pack()
        self.reset.pack(side=LEFT)
        self.QUIT.pack(side=LEFT, fill=BOTH)

    def __init__(self, master=None):
        Frame.__init__(self, master)
        Pack.config(self)
        self.createWidgets()


def callback_get_current_gains(current_PID_gains):
    global current_PID_gains_posture, current_PID_gains_linear_velocity, current_PID_gains_angular_velocity
    global DEFAULT_PID_GAINS_POSTURE, DEFAULT_PID_GAINS_LINEAR_VELOCITY, DEFAULT_PID_GAINS_ANGULAR_VELOCITY
    global new_PID_gains_posture, new_PID_gains_linear_velocity, new_PID_gains_angular_velocity

    current_PID_gains_posture = current_PID_gains.pid_gains_for_posture
    current_PID_gains_linear_velocity = current_PID_gains.pid_gains_for_linear_velocity
    current_PID_gains_angular_velocity = current_PID_gains.pid_gains_for_angular_velocity
    current_PID_gains_positon_control = current_PID_gains.pid_gains_for_position_control

    rospy.loginfo("current gains for posture : " +
                  str(current_PID_gains_posture))
    rospy.loginfo("current gains for linear velocity : " +
                  str(current_PID_gains_linear_velocity))
    rospy.loginfo("current gains for angular velocity : " +
                  str(current_PID_gains_angular_velocity))
    rospy.loginfo("current gains for position control : " +
                  str(current_PID_gains_positon_control))
    print("")


if __name__ == '__main__':
    time.sleep(1)
    rospy.init_node('set_param')
    pub_new_gains = rospy.Publisher(
        'new_PID_gains', PID_gains, queue_size=1)

    rospy.Subscriber('current_PID_gains', PID_gains,
                     callback_get_current_gains, queue_size=1)

    test = Test()
    test.mainloop()
