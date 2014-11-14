#!/usr/bin/env python
import rospy

from std_msgs.msg import String

class SaySomething(object):
    def __init__(self, speech):
        self._speech = speech

    def start_goal(self):
        msg = String(self._speech)
        self._publisher.publish(msg)

    def start(self):
        self._publisher = rospy.Publisher('/google_tts', String)

    def stop(self):
        self._publisher.unregister()

class MockSaySomething(object):
    def __init__(self, speech):
        self._speech = speech

    def start_goal(self):
        rospy.info('Mock say something: {}'.format(self._speech))

    def start(self):
        pass

    def stop(self):
        pass

