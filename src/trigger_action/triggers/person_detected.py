#!/usr/bin/env python

from people_msgs.msg import PositionMeasurement
import math
import rospy

class PersonDetected(object):
    TIMEOUT = 1 # How long to remember someone for, in seconds.
    # Meters a face can move between two messages to be considered the same
    # person.
    SAME_PERSON_DISTANCE = 0.1

    def __init__(self):
        self._last_update = None
        self._people = []
        self._callback = None # The callback for this trigger.

    def _face_detected_callback(self, position_measurement):
        if self._last_update is not None:
            duration = position_measurement.header.stamp - self._last_update
            if duration.to_sec() > PersonDetected.TIMEOUT:
                self._people = []
        self._last_update = position_measurement.header.stamp

        position = position_measurement.pos

        # Check if this face is far away from every previously seen face.
        for person_pos in self._people:
            if self._distance(position, person_pos) < PersonDetected.SAME_PERSON_DISTANCE:
                return
        if self._callback is not None:
            self._callback()
            
    def _distance(self, pos1, pos2):
        x = pos1.x - pos2.x
        y = pos1.y - pos2.y
        z = pos1.z - pos2.z
        return math.sqrt(x*x + y*y + z*z)

    def start(self):
        self._people_subscriber = rospy.Subscriber(
            '/face_detector/people_tracker_measurements',
            PositionMeasurement,
            self._face_detected_callback
        )
        self._last_update = None
        self._people = []

    def stop(self):
        self._people_subscriber.unregister()

    def set_callback(self, callback)
        self._callback = callback
