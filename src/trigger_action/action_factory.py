#!/usr/bin/env python
import json
from actions import say_something

def build(action_name, action_params):
    if action_name == 'say_something':
        params = json.loads(action_params)
        if 'speech' not in params:
            raise ValueError('"speech" parameter required for say_something.')
        speech = params['speech']
        trigger = person_detected.PersonDetected(speech)
        return trigger
    else:
        raise NotImplementedError
