#!/usr/bin/env python
import trigger_factory
import action_factory
import statement

def build(statement_msg, is_mock=False):
    trigger = trigger_factory.build(statement_msg.trigger_name,
        statement_msg.trigger_params, is_mock)
    action = action_factory.build(statement_msg.action_name,
        statement_msg.action_params, is_mock)
    statement_obj = statement.Statement(trigger, action)
    return statement_obj
