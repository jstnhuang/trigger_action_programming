#!/usr/bin/env python

class rule(object):
    def __init__(self, triggers, actions):
        self._triggers = triggers
        self._actions = actions

    def start(self):
        # TODO(jstn): implement multiple triggers 
        self._triggers[0].set_callback(self._execute_actions)
        for trigger in self._triggers:
            trigger.start()
        for action in self._actions:
            action.start()

    def _execute_actions(self):
        # TODO(jstn): implement multiple actions.
        action = self.actions[0]
        action.start_goal
        pass

    def stop(self):
        for trigger in self._triggers:
            trigger.stop()
        for action in self._actions:
            action.stop()
