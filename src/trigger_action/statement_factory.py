#!/usr/bin/env python
import trigger_factory
import action_factory
import statement

def build(statement_msg, is_mock=False):
    if is_mock:
        raise NotImplementedError
    trigger = trigger_factory.build(statement_msg.trigger_name,
        statement_msg.trigger_params)
    action = action_factory.build(statement_msg.action_name,
        statement_msg.action_params)
    statement_obj = statement.Statement(trigger, action)
    return statement_obj
