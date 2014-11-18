#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
import time as timemod

class TimeOfDay(object):
    def __init__(self, hour, minute, days):
        self._callback = None # The callback for this trigger.
        self._hour = hour
        self._minute = minute
        self._days = days
        # The event fires when we move from not being in the right minute
        # to being in the right minute.
        self._in_trigger_state = False

    def start(self):
        self._clock_subscriber = rospy.Subscriber(
          'clock', Time, self._clock_callback
        )

    def _clock_callback(self, time_msg):
        current_time = time_msg.data.to_sec()
        time_struct = timemod.localtime(current_time)
        in_state = (
            self._hour == time_struct.tm_hour
            and self._minute == time_struct.tm_min
        )
        if not self._in_trigger_state and in_state:
            self._callback()
        self._in_trigger_state = in_state

    def stop(self):
        self._clock_subscriber.unregister()

    def set_callback(self, callback):
        self._callback = callback
