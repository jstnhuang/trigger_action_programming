#!/usr/bin/env python
import node_factory
import rospy
import rostest
import unittest
from multiprocessing import Process
from trigger_action_programming.msg import Statement
from trigger_action_programming.srv import *

def statements_are_equal(a, b):
    return (
        a.trigger_name == b.trigger_name
        and a.trigger_params == b.trigger_params
        and a.action_name == b.action_name
        and a.action_params == b.action_params
    )

class NodeTest(unittest.TestCase):
    def setUp(self):
        def start_node():
            node = node_factory.build('trigger_action_programming', True)
            node.start()
        self._node_process = Process(target=start_node)
        self._node_process.start()

        rospy.wait_for_service('add_statement')
        self._add_statement = rospy.ServiceProxy('add_statement', AddStatement)
        rospy.wait_for_service('get_all_statements')
        self._get_all_statements = rospy.ServiceProxy('get_all_statements',
            GetAllStatements)
        rospy.wait_for_service('get_statement_by_id')
        self._get_statement_by_id = rospy.ServiceProxy('get_statement_by_id',
            GetStatementById)
        rospy.wait_for_service('update_statement')
        self._update_statement = rospy.ServiceProxy('update_statement',
            UpdateStatement)
        rospy.wait_for_service('delete_statement')
        self._delete_statement = rospy.ServiceProxy('delete_statement',
            DeleteStatement)

    def tearDown(self):
        self._node_process.terminate()

    def test_add_statement(self):
        statement = Statement(
            'unused',
            'person_detected', '{}',
            'say_something', '{"speech": "Hello"}'
        )
        request = AddStatementRequest(statement)
        response = self._add_statement(request)

        request = GetAllStatementsRequest()
        response = self._get_all_statements(request)
        statements = response.statements
        self.assertEquals(len(statements), 1)
        self.assertTrue(statements_are_equal(statements[0], statement)) # Add statement comparator that ignores ID.

    def test_get_by_id(self):
        statement = Statement(
            'unused',
            'person_detected', '{}',
            'say_something', '{"speech": "Hello"}'
        )
        request = AddStatementRequest(statement)
        id_response = self._add_statement(request)

        request = GetStatementByIdRequest(id_response.id)
        response = self._get_statement_by_id(request)
        self.assertTrue(statements_are_equal(statement, response.statement))

    def test_update(self):
        statement = Statement(
            'unused',
            'person_detected', '{}',
            'say_something', '{"speech": "Hello"}'
        )
        request = AddStatementRequest(statement)
        id_response = self._add_statement(request)

        request = GetStatementByIdRequest(id_response.id)
        response = self._get_statement_by_id(request)
        self.assertTrue(statements_are_equal(statement, response.statement))

        updated_statement = Statement(
            id_response.id,
            'person_detected', '{}',
            'say_something', '{"speech": "Hi there"}'
        )
        request = UpdateStatementRequest(id_response.id, updated_statement)
        self._update_statement(request)

        request = GetStatementByIdRequest(id_response.id)
        response = self._get_statement_by_id(request)
        self.assertTrue(statements_are_equal(updated_statement, response.statement))

    def test_delete(self):
        statement = Statement(
            'unused',
            'person_detected', '{}',
            'say_something', '{"speech": "Hello"}'
        )
        request = AddStatementRequest(statement)
        id_response = self._add_statement(request)

        request = GetAllStatementsRequest()
        response = self._get_all_statements(request)
        statements = response.statements
        self.assertEquals(len(statements), 1)

        request = DeleteStatementRequest(id_response.id)
        response = self._delete_statement(request)

        request = GetAllStatementsRequest()
        response = self._get_all_statements(request)
        statements = response.statements
        self.assertEquals(len(statements), 0)

if __name__ == '__main__':
    rostest.rosrun('trigger_action_programming', 'NodeTest', NodeTest)
