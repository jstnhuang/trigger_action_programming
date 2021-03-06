#!/usr/bin/env python
import json
from actions import say_something

def build(action_msg, is_mock=False):
    name = action_msg.name
    json_params = action_msg.params
    if name == 'say_something':
        params = json.loads(json_params)
        if 'content' not in params:
            raise ValueError('"content" parameter required for say_something.')
        speech = params['content']
        if is_mock:
            trigger = say_something.MockSaySomething(speech)
            return trigger
        else:
            trigger = say_something.SaySomething(speech)
            return trigger
    else:
        raise NotImplementedError
