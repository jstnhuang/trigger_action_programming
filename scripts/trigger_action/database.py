#!/usr/bin/env python
import rospy
import shelve

DATABASE_LOCATION = '/tmp/programs.db'

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

    def add_rule(self, rule_msg):
        rule_id = str(self.current_id())
        rule_msg.id = rule_id
        self._data[rule_id] = rule_msg
        self._increment_current_id()
        return rule_id

    def get_all_rules(self):
        return [v for k, v in self._data.items() if k != 'current_id']

    def get_rule_by_id(self, id):
        return self._data[id]

    def update_rule(self, id, rule_msg):
        rule_msg.id = id
        self._data[id] = rule_msg

    def delete_rule(self, id):
        del self._data[id]

    def close(self):
        self._data.close()

class InMemoryProgramDatabase(object):
    def __init__(self):
        self._data = {}
        self._data['current_id'] = 0

    def current_id(self):
        return self._data['current_id']

    def _increment_current_id(self):
        self._data['current_id'] += 1

    def add_rule(self, rule_msg):
        rule_id = str(self.current_id())
        rule_msg.id = rule_id
        self._data[rule_id] = rule_msg
        self._increment_current_id()
        return rule_id

    def get_all_rules(self):
        return [v for k, v in self._data.items() if k != 'current_id']

    def get_rule_by_id(self, id):
        return self._data[id]

    def update_rule(self, id, rule_msg):
        rule_msg.id = id
        self._data[id] = rule_msg

    def delete_rule(self, id):
        del self._data[id]

    def close(self):
        pass
