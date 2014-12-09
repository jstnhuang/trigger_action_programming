#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
import time as timemod

class TimeOfDay(object):
    def __init__(self, hour, minute, days):
        """Instantiate a new TimeOfDayTrigger

        Args:
          hour: The hour to trigger, in the range [0, 23]
          minute: The minute to trigger, in the range [0, 59]
          days: A list of lowercase weekday names for which this should trigger.
        """
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
        in_state = self._check_time(current_time)
        if not self._in_trigger_state and in_state:
            self._callback()
        self._in_trigger_state = in_state

    def _check_time(self, current_time):
        time_struct = timemod.localtime(current_time)
        days = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday']
        day = days[time_struct.tm_wday]
        return (
            self._hour == time_struct.tm_hour
            and self._minute == time_struct.tm_min
            and day in self._days
        )

    def stop(self):
        self._clock_subscriber.unregister()

    def set_callback(self, callback):
        self._callback = callback
