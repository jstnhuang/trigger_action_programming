#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class SaySomething(object):
    def __init__(self, speech):
        self._speech = speech
        self._client = SoundClient()

    def start_goal(self):
        self._client.say(self._speech, 'voice_us1_mbrola')

    def start(self):
        pass

    def stop(self):
        pass

class MockSaySomething(object):
    def __init__(self, speech):
        self._speech = speech

    def start_goal(self):
        rospy.info('Mock say something: {}'.format(self._speech))

    def start(self):
        pass

    def stop(self):
        pass

