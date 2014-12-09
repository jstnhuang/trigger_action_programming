#!/usr/bin/env python

from people_msgs.msg import PositionMeasurementArray
import math
import rospy

class PersonDetected(object):
    TIMEOUT = 1 # How long to remember someone for, in seconds.

    def __init__(self):
        self._last_update = None
        self._prev_num_people = 0
        self._callback = None # The callback for this trigger.

    def _face_detected_callback(self, position_measurement):
        if self._last_update is not None:
            duration = position_measurement.header.stamp - self._last_update
            if duration.to_sec() > PersonDetected.TIMEOUT:
              self._prev_num_people = 0
        self._last_update = position_measurement.header.stamp

        people = position_measurement.people
        if len(people) > self._prev_num_people:
            if self._callback is not None:
                self._callback()
            self._prev_num_people = len(people)
            
    def _distance(self, pos1, pos2):
        x = pos1.x - pos2.x
        y = pos1.y - pos2.y
        z = pos1.z - pos2.z
        return math.sqrt(x*x + y*y + z*z)

    def start(self):
        self._people_subscriber = rospy.Subscriber(
            '/face_detector/people_tracker_measurements_array',
            PositionMeasurementArray,
            self._face_detected_callback
        )
        self._last_update = None
        self._people = []

    def stop(self):
        self._people_subscriber.unregister()

    def set_callback(self, callback):
        self._callback = callback

class MockPersonDetected(object):
    def __init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def set_callback(self, callback):
        self._callback = callback

    def trigger(self):
        self._callback()
