#!/usr/bin/env python
import trigger_factory
import action_factory
import rule

def build(rule_msg, is_mock=False):
    triggers = []
    for trigger_msg in rule_msg.triggers:
        trigger = trigger_factory.build(trigger_msg, is_mock)
        triggers.append(trigger)
    actions = []
    for action_msg in rule_msg.actions:
        action = action_factory.build(action_msg, is_mock)
        actions.append(action)
    rule_obj = rule.rule(triggers, actions)
    return rule_obj
