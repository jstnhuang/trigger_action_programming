#!/usr/bin/python
from google.appengine.api import memcache
from google.appengine.ext import ndb
from google.appengine.ext import testbed
from flask import url_for
import json
import main
import questions
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

    def tearDown(self):
        self.testbed.deactivate()
        self.context.pop()

    def post_data(self, pkey, q):
        return json.dumps({
            'p': pkey.urlsafe(),
            'q': q,
            'rules': json.dumps([])
        })

    def testNextNeedsParticipantInDb(self):
        p = main.Participant()
        fake_key = ndb.Key('Participant', 'fake id')
        response = self.app.post('/next', data=self.post_data(fake_key, 0))
        self.assertEqual(response.data, url_for(
            'error', msg='Unknown participant.'))
        key = p.put()
        response = self.app.post('/next', data=self.post_data(key, 0))
        self.assertEqual(response.data, '/txg/webstudy.html?p={}&q={}'.format(
            key.urlsafe(), 1
        ))

    def testNextDoesNotInsertDuplicates(self):
        p = main.Participant()
        pkey = p.put()
        self.assertEqual(0, main.Response.query().count(2))
        response = self.app.post('/next', data=self.post_data(pkey, 0))
        self.assertEqual(response.data, '/txg/webstudy.html?p={}&q={}'.format(
            pkey.urlsafe(), 1
        ))
        self.assertEqual(1, main.Response.query().count(2))
        self.app.post('/next', data=self.post_data(pkey, 0))
        self.assertEqual(response.data, '/txg/webstudy.html?p={}&q={}'.format(
            pkey.urlsafe(), 1
        ))
        self.assertEqual(1, main.Response.query().count(2))

    def testNextCommonCase(self):
        p = main.Participant()
        pkey = p.put()
        num_q = len(questions.DEFAULT)
        # Insert all but the last response. Check that the database grows
        # accordingly, and that the redirect URL is correct.
        for i in range(num_q - 1):
            self.assertEqual(i, main.Response.query().count(num_q))
            response = self.app.post('/next', data=self.post_data(pkey, i))
            self.assertEqual(i+1, main.Response.query().count(num_q))
            self.assertEqual(
                response.data,
                '/txg/webstudy.html?p={}&q={}'.format(
                    pkey.urlsafe(), i+1
                )
            )

        # Check that no code exists yet.
        self.assertEqual(num_q - 1, main.Response.query().count(num_q))
        code = p.completion_code
        self.assertIsNone(code)

        # Insert the last response.
        response = self.app.post('/next', data=self.post_data(pkey, num_q-1))
        self.assertEqual(num_q, main.Response.query().count(num_q+1))

        # Check that the completion code exists.
        code = p.completion_code
        self.assertIsNotNone(code)
        self.assertEqual(
            response.data,
            url_for('end', code=code)
        )

        # Check that only one completion code is generated, even if the last
        # response is submitted multiple times.
        response = self.app.post('/next', data=self.post_data(pkey, num_q-1))
        self.assertEqual(num_q, main.Response.query().count(num_q+1))
        code2 = p.completion_code
        self.assertEqual(code, code2)
        self.assertEqual(
            response.data,
            url_for('end', code=code)
        )

    def testNextIfIncomplete(self):
        p = main.Participant()
        pkey = p.put()
        num_q = len(questions.DEFAULT)
        self.assertEqual(0, main.Response.query().count(2))
        response = self.app.post('/next', data=self.post_data(pkey, num_q-1))
        self.assertEqual(
            response.data,
            url_for('error', msg='The responses were incomplete.')
        )

    def testStartInsertsParticipant(self):
        self.assertEqual(0, main.Participant.query().count(1))
        self.app.get('/start')
        self.assertEqual(1, main.Participant.query().count(1))


if __name__ == '__main__':
    unittest.main()
