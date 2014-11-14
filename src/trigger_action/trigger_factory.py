#!/usr/bin/env python
import json
import rospy
from triggers import person_detected

def build(trigger_name, trigger_params, is_mock=False):
    if trigger_name == 'person_detected':
        if is_mock:
            trigger = person_detected.MockPersonDetected()
            return trigger
        else:
            trigger = person_detected.PersonDetected()
            return trigger
    else:
        raise NotImplementedError
