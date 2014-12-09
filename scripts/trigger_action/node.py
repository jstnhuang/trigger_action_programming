#!/usr/bin/env python
import node_factory
import rospy
import rule_factory
from trigger_action_programming.srv import *

class Node(object):
    def __init__(self, name, db, is_mock):
        self._name = name
        self._db = db
        self._is_mock = is_mock

    def start(self):
        rospy.init_node(self._name)
        self._rules = {}
        self._load_programs()
        self._add_service = rospy.Service('add_rule',
            AddRule, self._handle_add_rule)
        self._get_all_service = rospy.Service('get_all_rules',
            GetAllRules, self._handle_get_all_rules)
        self._get_by_id_service = rospy.Service('get_rule_by_id',
            GetRuleById, self._handle_get_rule_by_id)
        self._update_service = rospy.Service('update_rule',
            UpdateRule, self._handle_update_rule)
        self._delete_service = rospy.Service('delete_rule',
            DeleteRule, self._handle_delete_rule)
        # In-memory version of database with constructed Rule objects.
        rospy.on_shutdown(self._handle_shutdown)
        rospy.spin()

    def _load_programs(self):
        rule_msgs = self._db.get_all_rules()
        for rule_msg in rule_msgs:
            rule = rule_factory.build(rule_msg, self._is_mock)
            self._rules[rule_msg.id] = rule
            rule.start()

    def _handle_shutdown(self):
        self._db.close()

    def _handle_add_rule(self, request):
        rule_id = self._db.add_rule(request.rule)
        rule = rule_factory.build(request.rule, self._is_mock)
        self._rules[rule_id] = rule
        rule.start()
        response = AddRuleResponse(rule_id)
        return response

    def _handle_get_all_rules(self, request):
        rules = self._db.get_all_rules()
        response = GetAllRulesResponse(rules)
        return response

    def _handle_get_rule_by_id(self, request):
        try:
            rule = self._db.get_rule_by_id(request.id)
            response = GetRuleByIdResponse(rule)
            return response
        except KeyError:
            raise rospy.ServiceException('No rule with ID: {}'.format(request.id))

    def _handle_update_rule(self, request):
        try:
            self._db.update_rule(request.id, request.updated_rule)
            response = UpdateRuleResponse()

            rule = rule_factory.build(request.updated_rule, self._is_mock)
            self._rules[request.id].stop()
            self._rules[request.id] = rule
            rule.start()

            return response
        except KeyError:
            raise rospy.ServiceException('No rule with ID: {}'.format(request.id))

    def _handle_delete_rule(self, request):
        try:
            self._db.delete_rule(request.id)
            response = DeleteRuleResponse()
            self._rules[request.id].stop()
            del self._rules[request.id]
            return response
        except KeyError:
            raise rospy.ServiceException('No rule with ID: {}'.format(request.id))

class DatabaseOnlyNode(object):
    """A node that only saves and loads programs, without running them.
    """
    def __init__(self, name, db):
        self._name = name
        self._db = db

    def start(self):
        rospy.init_node(self._name)
        self._add_service = rospy.Service('add_rule',
            AddRule, self._handle_add_rule)
        self._get_all_service = rospy.Service('get_all_rules',
            GetAllRules, self._handle_get_all_rules)
        self._get_by_id_service = rospy.Service('get_rule_by_id',
            GetRuleById, self._handle_get_rule_by_id)
        self._update_service = rospy.Service('update_rule',
            UpdateRule, self._handle_update_rule)
        self._delete_service = rospy.Service('delete_rule',
            DeleteRule, self._handle_delete_rule)
        # In-memory version of database with constructed Rule objects.
        rospy.on_shutdown(self._handle_shutdown)
        rospy.spin()

    def _handle_shutdown(self):
        self._db.close()

    def _handle_add_rule(self, request):
        rule_id = self._db.add_rule(request.rule)
        response = AddRuleResponse(rule_id)
        return response

    def _handle_get_all_rules(self, request):
        rules = self._db.get_all_rules()
        response = GetAllRulesResponse(rules)
        return response

    def _handle_get_rule_by_id(self, request):
        try:
            rule = self._db.get_rule_by_id(request.id)
            response = GetRuleByIdResponse(rule)
            return response
        except KeyError:
            raise rospy.ServiceException('No rule with ID: {}'.format(request.id))

    def _handle_update_rule(self, request):
        try:
            self._db.update_rule(request.id, request.updated_rule)
            response = UpdateRuleResponse()

            return response
        except KeyError:
            raise rospy.ServiceException('No rule with ID: {}'.format(request.id))

    def _handle_delete_rule(self, request):
        try:
            self._db.delete_rule(request.id)
            response = DeleteRuleResponse()
            return response
        except KeyError:
            raise rospy.ServiceException('No rule with ID: {}'.format(request.id))

if __name__ == '__main__':
    is_mock = rospy.get_param('trigger_action_programming/mock')
    main_node = node_factory.build('trigger_action_programming', is_mock)
    main_node.start()
