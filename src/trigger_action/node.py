#!/usr/bin/env python
import node_factory
import rospy
import statement_factory.py
from trigger_action_programming.srv import *

class Node(object):
    def __init__(self, name, db):
        self._name = name
        self._db = db

    def start(self):
        rospy.init_node(self._name)
        self._add_service = rospy.Service('add_statement',
            AddStatement, self._handle_add_statement)
        self._get_all_service = rospy.Service('get_all_statements',
            GetAllStatements, self._handle_get_all_statements)
        self._get_by_id_service = rospy.Service('get_statement_by_id',
            GetStatementById, self._handle_get_statement_by_id)
        self._update_service = rospy.Service('update_statement',
            UpdateStatement, self._handle_update_statement)
        self._delete_service = rospy.Service('delete_statement',
            DeleteStatement, self._handle_delete_statement)
        # In-memory version of database with constructed Statement objects.
        self._statements = {}
        rospy.spin()

    def _handle_add_statement(self, request):
        statement_id = self._db.add_statement(request.statement)
        statement = statement_factory.build(request.statement)
        self._statements[statement_id] = statement
        statement.start()
        return statement_id

    def _handle_get_all_statements(self, request):
        statements = self._db.get_all_statements()
        response = GetAllStatementsResponse(statements)
        return response

    def _handle_get_statement_by_id(self, request):
        try:
            statement = self._db.get_statement_by_id(request.id)
            response = GetStatementByIdResponse(statement)
            return response
        except KeyError:
            raise rospy.ServiceException('No statement with ID: {}'.format(id))

    def _handle_update_statement(self, request):
        try:
            self._db.update_statement(request.id, request.updated_statement)
            response = UpdateStatementResponse()

            statement = statement_factory.build(request.updated_statement)
            self._statements[statement_id].stop()
            self._statements[statement_id] = statement
            statement.start()

            return response
        except KeyError:
            raise rospy.ServiceException('No statement with ID: {}'.format(id))

    def _handle_delete_statement(self, request):
        try:
            statement_id = self._db.delete_statement(request.id)
            response = DeleteStatementResponse()
            return response
        except KeyError:
            raise rospy.ServiceException('No statement with ID: {}'.format(id))

if __name__ == '__main__':
    main_node = node_factory.build('trigger_action_programming')
    main_node.start()
