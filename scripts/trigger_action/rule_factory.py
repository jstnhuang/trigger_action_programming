#!/usr/bin/env python
import trigger_factory
import action_factory
import rule

def build(rule_msg, is_mock=False):
    trigger = trigger_factory.build(rule_msg.trigger_name,
        rule_msg.trigger_params, is_mock)
    action = action_factory.build(rule_msg.action_name,
        rule_msg.action_params, is_mock)
    rule_obj = rule.rule(trigger, action)
    return rule_obj
