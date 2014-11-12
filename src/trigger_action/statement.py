#!/usr/bin/env python

class Statement(object):
    def __init__(self, trigger, action):
        self._trigger = trigger
        self._action = action

    def start(self):
        self._trigger.set_callback(self._action.start_goal)
        self._trigger.start()
        self._action.start()

    def stop(self):
        self._trigger.stop()
        self._action.stop()
