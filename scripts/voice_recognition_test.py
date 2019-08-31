#!/usr/bin/env python
# -*- coding: utf-8 -*-
import click
import rospy
from std_msgs.msg import String
from module import google_assistant


@click.command()
@click.option("--lang", default="en-US")
@click.option("--debug", default=False)
@click.option("--answer", default=False)
def start(lang, debug, answer):
    def callback(data):
        rospy.loginfo("Recognition Start")
        result, answer = assistant.start()
        pub.publish(result)
        rospy.loginfo("Result -> %s" % result)
        r.sleep()

    rospy.init_node('voice_recognition', anonymous=False)
    rospy.Subscriber("voice_recognition/start", String, callback)
    pub = rospy.Publisher('voice_recognition/result', String, queue_size=10)
    r = rospy.Rate(10)

    assistant = google_assistant.main(lang, debug, answer)

    rospy.spin()


if __name__ == '__main__':
    start()
