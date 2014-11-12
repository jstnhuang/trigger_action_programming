#!/usr/bin/env python
import shelve

DATABASE_LOCATION = 'programs.db'

class ProgramDatabase(object):
    def __init__(self, path):
        self._path = path
        self._data = shelve.open(path)
        if 'current_id' not in self._data:
            self._data['current_id'] = 0

    def current_id(self):
        return self._data['current_id']

    def _increment_current_id(self):
        self._data['current_id'] += 1

    def add_statement(self, statement_msg):
        statement_id = str(self.current_id())
        self._data[statement_id] = statement_msg
        self._increment_current_id()
        return statement_id

    def get_all_statements(self):
        return [v for k, v in self._data.items() if k != 'current_id']

    def get_statement_by_id(self, id):
        return self._data[id]

    def update_statement(self, id, statement_msg):
        self._data[id] = statement_msg

    def delete_statement(self, id):
        del self._data[id]

class InMemoryProgramDatabase(object):
    def __init__(self):
        self._data = {}
        self._data['current_id'] = 0

    def current_id(self):
        return self._data['current_id']

    def _increment_current_id(self):
        self._data['current_id'] += 1

    def add_statement(self, statement_msg):
        statement_id = str(self.current_id())
        self._data[statement_id] = statement_msg
        self._increment_current_id()
        return statement_id

    def get_all_statements(self):
        return [v for k, v in self._data.items() if k != 'current_id']

    def get_statement_by_id(self, id):
        return self._data[id]

    def update_statement(self, id, statement_msg):
        self._data[id] = statement_msg

    def delete_statement(self, id):
        del self._data[id]
