#!/usr/bin/env python
import json
import rospy
from triggers import person_detected

def build(trigger_name, trigger_params):
    if trigger_name == 'person_detected':
        trigger = person_detected.PersonDetected()
        return trigger
    else:
        raise NotImplementedError
