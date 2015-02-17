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
        self.testbed.setup_env(USER_EMAIL='usermail@gmail.com', USER_ID='1', USER_IS_ADMIN='1')
        self.testbed.activate()
        self.testbed.init_datastore_v3_stub()
        self.testbed.init_memcache_stub()
        self.testbed.init_user_stub()

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

    def testGetResponsesExcludesIncomplete(self):
        e_key = main.experiment_key('test_experiment')
        p1 = main.Participant(completion_code=123456, parent=e_key)
        p2 = main.Participant(parent=e_key)
        p1_key = p1.put()
        p2_key = p2.put()
        response1 = main.Response(participant_key=p1_key, question_id=5, selected='valid', parent=e_key)
        response2 = main.Response(participant_key=p2_key, question_id=5, selected='invalid', parent=e_key)
        r1_key = response1.put()
        r2_key = response2.put()
        response = self.app.get('/admin_api/get_responses/test_experiment')
        data = json.loads(response.data)
        self.assertDictEqual({'valid': 1}, data[5])

    def testGetResponsesJsonSort(self):
        e_key = main.experiment_key('test_experiment')
        p1 = main.Participant(completion_code=123456, parent=e_key)
        p2 = main.Participant(completion_code=234567, parent=e_key)
        p1_key = p1.put()
        p2_key = p2.put()
        rules1 = [
            {
                'triggers': [
                    {
                        'key': 'weather',
                        'displayName': 'Daily time',
                        'params': {
                            'endHour': 10,
                            'endMinute': 0,
                            'isOrBetween': 'is between',
                            'startHour': 7,
                            'startMinute': 0
                        }
                    },
                    {
                        'key': 'my_location',
                        'displayName': 'My location',
                        'params': {
                            'verb': 'arrive at',
                            'location': 'home'
                        }
                    },
                ],
                'action': {
                    'key': 'brew_coffee',
                    'displayName': 'Brew coffee',
                    'params': {}
                }
            }
        ]
        rules2 = [
            {
                'triggers': [
                    {
                        'key': 'my_location',
                        'displayName': 'My location',
                        'params': {
                            'verb': 'arrive at',
                            'location': 'home'
                        }
                    },
                    {
                        'key': 'weather',
                        'displayName': 'Daily time',
                        'params': {
                            'endHour': 10,
                            'endMinute': 0,
                            'isOrBetween': 'is between',
                            'startHour': 7,
                            'startMinute': 0
                        }
                    },
                ],
                'action': {
                    'key': 'brew_coffee',
                    'displayName': 'Brew coffee',
                    'params': {}
                }
            }
        ]
        json_rules1 = json.dumps(rules1, sort_keys=True)
        json_rules2 = json.dumps(rules2, sort_keys=True)
        response1 = main.Response(participant_key=p1_key, rules=json_rules1, question_id=1, parent=e_key)
        response2 = main.Response(participant_key=p2_key, rules=json_rules2, question_id=1, parent=e_key)
        r1_key = response1.put()
        r2_key = response2.put()
        response = self.app.get('/admin_api/get_responses/test_experiment')
        data = json.loads(response.data)
        self.assertDictEqual({json_rules1: 2}, data[1])


if __name__ == '__main__':
    unittest.main()
