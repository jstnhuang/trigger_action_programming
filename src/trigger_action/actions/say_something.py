#!/usr/bin/env python

from std_msgs.msg import String
class SaySomething(object):
    def __init__(self, speech):
        self._speech = speech
        self._publisher = rospy.Publisher('/google_tts', String)

    def start_goal():
        msg = String(self._speech)
        self._publisher.publish(msg)
