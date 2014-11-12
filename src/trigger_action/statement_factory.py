#!/usr/bin/env python
import trigger_factory
import action_factory
import statement

def build(statement, is_mock=True):
    if is_mock:
        raise NotImplementedError
    trigger = trigger_factory.build(statement.trigger_name,
        statement.trigger_params)
    action = action_factory.build(statement.action_name,
        statement.action_params)
    statement = Statement(trigger, action)
    return statement
