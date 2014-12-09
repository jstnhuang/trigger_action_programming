#!/usr/bin/env python
import json
import rospy
from triggers import person_detected
from triggers import time_of_day

def build(trigger_msg, is_mock=False):
    name = trigger_msg.name
    json_params = trigger_msg.params
    params = json.loads(json_params)
    if name == 'person_detected':
        if is_mock:
            trigger = person_detected.MockPersonDetected()
            return trigger
        else:
            trigger = person_detected.PersonDetected()
            return trigger
    elif name == 'time_of_day':
        if 'hour' not in params or 'minute' not in params or 'days' not in params:
            raise ValueError('"hour", "minute", and/or "days" parameter(s) missing.')
        hour = int(params['hour'])
        minute = int(params['minute'])
        days = set(params['days'])
        trigger = time_of_day.TimeOfDay(hour, minute, days)
        return trigger
    else:
        raise NotImplementedError
