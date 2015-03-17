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
        if ('startHour' not in params or
                'startMinute' not in params or
                'sunday' not in params or
                'monday' not in params or
                'tuesday' not in params or
                'wednesday' not in params or
                'thursday' not in params or
                'friday' not in params or
                'saturday' not in params):
            raise ValueError('"startHour", "startMinute", and/or "sunday", '
                '"monday", ..., "saturday" parameter(s) missing.')
        hour = int(params['startHour'])
        minute = int(params['startMinute'])
        day_list = ['sunday', 'monday', 'tuesday', 'wednesday', 'thursday',
            'friday', 'saturday']
        days = set([day in day_list if params[day]);
        trigger = time_of_day.TimeOfDay(hour, minute, days)
        return trigger
    else:
        raise NotImplementedError
