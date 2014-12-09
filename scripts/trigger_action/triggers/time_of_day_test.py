#!/usr/bin/env python

from std_msgs.msg import Time
import datetime
import rospy
import rostest
import unittest
import time_of_day

# TODO(jstn): Move this to a module where it can be reused.
class MockCallback(object):
    def __init__(self):
        self.num_calls = 0
        self.was_called = False

    def __call__(self):
        self.num_calls += 1
        self.was_called = True

class MockTimePublisher(object):
    def __init__(self, trigger):
      self._trigger = trigger

    def _make_time(self, day, hour, minute, seconds):
        days = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday']
        day_index = days.index(day)
        # 2014/11/17 was a Monday.
        dt = datetime.datetime(2014, 11, 17 + day_index, hour, minute, seconds)
        seconds = float(dt.strftime('%s'))
        return rospy.Time(seconds)

    def publish(self, day, hour, minute, seconds):
        time = self._make_time(day, hour, minute, seconds)
        self._trigger._clock_callback(Time(time))

class TimeOfDayTriggerTest(unittest.TestCase):
    def _publish_time(self, day, hour, minute, seconds):
        time = self._make_time(day, hour, minute, seconds) 
        self._clock_publisher.publish(time)

    def test_time_of_day_trigger(self):
        trigger = time_of_day.TimeOfDay(13, 30, ['monday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('monday', 13, 29, 59) 
        self.assertFalse(callback.was_called)
        mock_time_publisher.publish('monday', 13, 30, 00) 
        self.assertTrue(callback.was_called)

    def test_no_double_trigger(self):
        trigger = time_of_day.TimeOfDay(13, 30, ['monday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('monday', 13, 30, 00) 
        self.assertEqual(callback.num_calls, 1)
        mock_time_publisher.publish('monday', 13, 30, 01) 
        self.assertEqual(callback.num_calls, 1)

    def test_wraparound(self):
        trigger = time_of_day.TimeOfDay(13, 30, ['monday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('monday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 1)
        mock_time_publisher.publish('monday', 13, 31, 00) 
        self.assertEquals(callback.num_calls, 1)
        mock_time_publisher.publish('monday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 2)

    def test_immediate_trigger(self):
        trigger = time_of_day.TimeOfDay(13, 30, ['monday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('monday', 13, 30, 00) 
        self.assertTrue(callback.was_called)

    def test_midnight_trigger(self):
        trigger = time_of_day.TimeOfDay(0, 0, ['monday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('sunday', 23, 59, 59) 
        self.assertFalse(callback.was_called)
        mock_time_publisher.publish('monday', 0, 0, 0) 
        self.assertTrue(callback.was_called)

    def test_wrong_day(self):
        trigger = time_of_day.TimeOfDay(13, 30, ['monday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('tuesday', 23, 59, 59) 
        self.assertFalse(callback.was_called)

    def test_multiple_days(self):
        trigger = time_of_day.TimeOfDay(13, 30, ['monday', 'tuesday', 'wednesday']) 
        mock_time_publisher = MockTimePublisher(trigger)
        callback = MockCallback()
        trigger.set_callback(callback)

        mock_time_publisher.publish('sunday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 0)
        mock_time_publisher.publish('monday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 1)
        mock_time_publisher.publish('tuesday', 13, 29, 59) 
        mock_time_publisher.publish('tuesday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 2)
        mock_time_publisher.publish('wednesday', 13, 29, 59) 
        mock_time_publisher.publish('wednesday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 3)
        mock_time_publisher.publish('thursday', 13, 29, 59) 
        mock_time_publisher.publish('thursday', 13, 30, 00) 
        self.assertEquals(callback.num_calls, 3)
        
if __name__ == '__main__':
    rostest.rosrun(
        'trigger_action_programming', 'TimeOfDayTriggerTest', TimeOfDayTriggerTest)
