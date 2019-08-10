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

current_PID_gains_posture = [0, 0, 0]
current_PID_gains_linear_velocity = [0, 0, 0]
current_PID_gains_angular_velocity = [0, 0, 0]

DEFAULT_PID_GAINS_POSTURE = [0, 0, 0]
DEFAULT_PID_GAINS_LINEAR_VELOCITY = [0, 0, 0]
DEFAULT_PID_GAINS_ANGULAR_VELOCITY = [0, 0, 0]

new_PID_gains_posture = [0, 0, 0]
new_PID_gains_linear_velocity = [0, 0, 0]
new_PID_gains_angular_velocity = [0, 0, 0]


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

    def createWidgets(self):
        # scale for posture control gains
        self.slider_Pgain_pos = Scale(self, from_=0, to=15000,
                                      # value=50,
                                      orient=HORIZONTAL,
                                      length="8i",
                                      label="P gain for POS",
                                      command=self.set_Pgain_for_posture)
        self.slider_Pgain_pos.set(current_PID_gains_posture[0])

        self.slider_Igain_pos = Scale(self, from_=0, to=100,
                                      # value=50,
                                      orient=HORIZONTAL,
                                      length="8i",
                                      label="I gain for POS",
                                      command=self.set_Igain_for_posture)
        self.slider_Igain_pos.set(current_PID_gains_posture[1])

        self.slider_Dgain_pos = Scale(self, from_=0, to=60000,
                                      # value=50,
                                      orient=HORIZONTAL,
                                      length="8i",
                                      label="D gain for POS",
                                      command=self.set_Dgain_for_posture)
        self.slider_Dgain_pos.set(current_PID_gains_posture[2])

        # scale for linear velocity control gains
        self.slider_Pgain_linVel = Scale(self, from_=0, to=1000,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="P gain for linVel",
                                         command=self.set_Pgain_for_linear_velocity)
        self.slider_Pgain_linVel.set(current_PID_gains_linear_velocity[0])

        self.slider_Igain_linVel = Scale(self, from_=0, to=1000,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="I gain for linVel",
                                         command=self.set_Igain_for_linear_velocity)
        self.slider_Igain_linVel.set(current_PID_gains_linear_velocity[1])

        self.slider_Dgain_linVel = Scale(self, from_=0, to=1000,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="D gain for linVel",
                                         command=self.set_Dgain_for_linear_velocity)
        self.slider_Dgain_linVel.set(current_PID_gains_linear_velocity[2])

        # scale for angular velocity control gains
        self.slider_Pgain_angVel = Scale(self, from_=0, to=1000,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="P gain for angVel",
                                         command=self.set_Pgain_for_angular_velocity)
        self.slider_Pgain_angVel.set(current_PID_gains_angular_velocity[0])

        self.slider_Igain_angVel = Scale(self, from_=0, to=1000,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="I gain for angVel",
                                         command=self.set_Igain_for_angular_velocity)
        self.slider_Igain_angVel.set(current_PID_gains_angular_velocity[1])

        self.slider_Dgain_angVel = Scale(self, from_=0, to=1000,
                                         # value=50,
                                         orient=HORIZONTAL,
                                         length="8i",
                                         label="D gain for angVel",
                                         command=self.set_Dgain_for_angular_velocity)
        self.slider_Dgain_angVel.set(current_PID_gains_angular_velocity[2])

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

    DEFAULT_PID_GAINS_POSTURE = current_PID_gains_posture
    DEFAULT_PID_GAINS_LINEAR_VELOCITY = current_PID_gains_linear_velocity
    DEFAULT_PID_GAINS_ANGULAR_VELOCITY = current_PID_gains_angular_velocity

    new_PID_gains_posture = current_PID_gains_posture
    new_PID_gains_linear_velocity = current_PID_gains_linear_velocity
    new_PID_gains_angular_velocity = current_PID_gains_angular_velocity

    rospy.loginfo("current gains for posture : " +
                  str(current_PID_gains_posture))
    rospy.loginfo("current gains for linear velocity : " +
                  str(current_PID_gains_linear_velocity))
    rospy.loginfo("current gains for angular velocity : " +
                  str(current_PID_gains_angular_velocity))
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
