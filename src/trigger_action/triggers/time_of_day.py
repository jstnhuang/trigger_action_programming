#!/usr/bin/env python

import rospy
import time as timemod

class TimeOfDay(object):
    def __init__(self, time_provider, hour, minute, days):
        self._callback = None # The callback for this trigger.
        self._time_provider = time_provider
        self._is_active = False
        self._hour = hour
        self._minute = minute
        self._days = days
        # When the time turns to the given time/hour, sends 
        self._in_trigger_state = False

    def start(self):
        rate = rospy.Rate(1)
        self._is_active = True
        while self._is_active:
            self._check_state()
            rate.sleep()

    def _check_state(self):
        current_time = self._time_provider.time()
        time_struct = timemod.localtime(current_time)
        in_state = (
            self._hour == time_struct.tm_hour
            and self._minute == time_struct.tm_min
        )
        if not self._in_trigger_state and in_state:
            self._callback()
        self._in_trigger_state = in_state

    def _check_time(self, hour, minute):
        return self._hour == hour and self._minute == minute

    def stop(self):
        self._is_active = False
        self._people_subscriber.unregister()

    def set_callback(self, callback):
        self._callback = callback

class RosTimeProvider(object):
    def __init__(self):
        pass

    def get_time(self):
        return rospy.get_time()

class MockTimeProvider(object):
    def __init__(self):
        self._time = 0

    def set_time(self, time):
        self._time = time

    def get_time(self):
        return time
