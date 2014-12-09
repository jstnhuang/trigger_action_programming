#!/usr/bin/env python
import json
from actions import say_something

def build(action_name, action_params, is_mock=False):
    if action_name == 'say_something':
        params = json.loads(action_params)
        if 'speech' not in params:
            raise ValueError('"speech" parameter required for say_something.')
        speech = params['speech']
        if is_mock:
            trigger = say_something.MockSaySomething(speech)
            return trigger
        else:
            trigger = say_something.SaySomething(speech)
            return trigger
    else:
        raise NotImplementedError
