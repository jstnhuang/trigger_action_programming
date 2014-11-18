#!/usr/bin/env python
import json
import rospy
from triggers import person_detected
from triggers import time_of_day

def build(trigger_name, trigger_params, is_mock=False):
    params = json.loads(trigger_params)
    if trigger_name == 'person_detected':
        if is_mock:
            trigger = person_detected.MockPersonDetected()
            return trigger
        else:
            trigger = person_detected.PersonDetected()
            return trigger
    elif trigger_name == 'time_of_day':
        time_provider = None
        if is_mock:
            time_provider = time_of_day.MockTimeProvider()
        else:
            time_provider = time_of_day.RosTimeProvider()
        if 'hour' not in params or 'minute' not in params or 'days' not in params:
            raise ValueError('"hour", "minute", and/or "days" parameter(s) missing.')
        hour = int(params['hour'])
        minute = int(params['minute'])
        days = set(params['days'])
        trigger = time_of_day.TimeOfDay(time_provider, hour, minute, days)
        return trigger
    else:
        raise NotImplementedError
