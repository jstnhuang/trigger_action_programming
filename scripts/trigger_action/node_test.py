#!/usr/bin/env python
import node_factory
import rospy
import rostest
import unittest
from multiprocessing import Process
from trigger_action_programming.msg import *
from trigger_action_programming.srv import *

def triggers_are_equal(a, b):
    return a.name == b.name and a.params == b.params

def actions_are_equal(a, b):
    return a.name == b.name and a.params == b.params

def rules_are_equal(a, b):
    for ta, tb in zip(sorted(a.triggers), sorted(b.triggers)):
        if not triggers_are_equal(ta, tb):
            return False
    for aa, ab in zip(sorted(a.actions), sorted(b.actions)):
        if not actions_are_equal(aa, ab):
            return False
    return True

class NodeTest(unittest.TestCase):
    def setUp(self):
        def start_node():
            node = node_factory.build('trigger_action_programming', True)
            node.start()
        self._node_process = Process(target=start_node)
        self._node_process.start()

        rospy.wait_for_service('add_rule')
        self._add_rule = rospy.ServiceProxy('add_rule', AddRule)
        rospy.wait_for_service('get_all_rules')
        self._get_all_rules = rospy.ServiceProxy('get_all_rules',
            GetAllRules)
        rospy.wait_for_service('get_rule_by_id')
        self._get_rule_by_id = rospy.ServiceProxy('get_rule_by_id',
            GetRuleById)
        rospy.wait_for_service('update_rule')
        self._update_rule = rospy.ServiceProxy('update_rule',
            UpdateRule)
        rospy.wait_for_service('delete_rule')
        self._delete_rule = rospy.ServiceProxy('delete_rule',
            DeleteRule)

    def tearDown(self):
        self._node_process.terminate()

    def test_add_rule(self):
        rule = Rule(
            'unused',
            [Trigger('person_detected', '{}')],
            [Action('say_something', '{"speech": "Hello"}')]
        )
        request = AddRuleRequest(rule)
        response = self._add_rule(request)

        request = GetAllRulesRequest()
        response = self._get_all_rules(request)
        rules = response.rules
        self.assertEquals(len(rules), 1)
        self.assertTrue(rules_are_equal(rules[0], rule))
        self.assertEquals(rules[0].id, '0')

    def test_get_by_id(self):
        rule = Rule(
            'unused',
            [Trigger('person_detected', '{}')],
            [Action('say_something', '{"speech": "Hello"}')]
        )
        request = AddRuleRequest(rule)
        id_response = self._add_rule(request)

        request = GetRuleByIdRequest(id_response.id)
        response = self._get_rule_by_id(request)
        self.assertTrue(rules_are_equal(rule, response.rule))

    def test_get_bad_id(self):
        try:
            request = GetRuleByIdRequest('badid')
            response = self._get_rule_by_id(request)
        except rospy.ServiceException:
            return

    def test_update(self):
        rule = Rule(
            'unused',
            [Trigger('person_detected', '{}')],
            [Action('say_something', '{"speech": "Hello"}')]
        )
        request = AddRuleRequest(rule)
        id_response = self._add_rule(request)

        request = GetRuleByIdRequest(id_response.id)
        response = self._get_rule_by_id(request)
        self.assertTrue(rules_are_equal(rule, response.rule))

        updated_rule = Rule(
            'unused',
            [Trigger('person_detected', '{}')],
            [Action('say_something', '{"speech": "Hi there"}')]
        )
        request = UpdateRuleRequest(id_response.id, updated_rule)
        self._update_rule(request)

        request = GetRuleByIdRequest(id_response.id)
        response = self._get_rule_by_id(request)
        self.assertTrue(rules_are_equal(updated_rule, response.rule))
        self.assertEquals(response.rule.id, id_response.id)

    def test_update_bad_id(self):
        try:
            rule = Rule(
                'unused',
                [Trigger('person_detected', '{}')],
                [Action('say_something', '{"speech": "Hello"}')]
            )
            request = UpdateRuleRequest('badid', rule)
            response = self._update_rule(request)
        except rospy.ServiceException:
            return

    def test_delete(self):
        rule = Rule(
            'unused',
            [Trigger('person_detected', '{}')],
            [Action('say_something', '{"speech": "Hello"}')]
        )
        request = AddRuleRequest(rule)
        id_response = self._add_rule(request)

        request = GetAllRulesRequest()
        response = self._get_all_rules(request)
        rules = response.rules
        self.assertEquals(len(rules), 1)

        request = DeleteRuleRequest(id_response.id)
        response = self._delete_rule(request)

        request = GetAllRulesRequest()
        response = self._get_all_rules(request)
        rules = response.rules
        self.assertEquals(len(rules), 0)

    def test_delete_bad_id(self):
        try:
            request = DeleteRuleRequest('badid')
            response = self._delete_rule(request)
        except rospy.ServiceException:
            return

if __name__ == '__main__':
    rostest.rosrun('trigger_action_programming', 'NodeTest', NodeTest)
