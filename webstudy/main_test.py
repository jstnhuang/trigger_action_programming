#!/usr/bin/python
from google.appengine.api import memcache
from google.appengine.ext import ndb
from google.appengine.ext import testbed
from flask import url_for
import json
import main
import unittest


class MainTest(unittest.TestCase):
    def setUp(self):
        self.app = main.app.test_client()
        self.context = main.app.test_request_context()
        self.context.push()
        self.testbed = testbed.Testbed()
        self.testbed.activate()
        self.testbed.init_datastore_v3_stub()
        self.testbed.init_memcache_stub()
        self._TEST_EXPERIMENT_KEY = ndb.Key('Experiment', 'MainTest')

    def tearDown(self):
        self.testbed.deactivate()
        self.context.pop()

    def testNextNeedsParticipantInDb(self):
        p = main.Participant()
        response = self.app.post('/next', data=json.dumps({
            'p': ndb.Key('Participant', 'fake_participant_key',
                         parent=self._TEST_EXPERIMENT_KEY).urlsafe(),
            'q': 0,
            'rules': json.dumps([])
        }))
        self.assertEqual(response.data, url_for(
            'error', msg='Unknown participant.'))
        key = p.put()
        response = self.app.post('/next', data=json.dumps({
            'p': key.urlsafe(),
            'q': 0,
            'rules': json.dumps([])
        }))
        self.assertEqual(response.data, '/txg/webstudy.html?p={}&q={}'.format(
            key.urlsafe(), 1
        ))

    def testStartInsertsParticipant(self):
        self.assertEqual(0, main.Participant.query().count(1))
        self.app.get('/start')
        self.assertEqual(1, main.Participant.query().count(1))


if __name__ == '__main__':
    unittest.main()
