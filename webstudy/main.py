from flask import Flask
from flask import make_response
from flask import redirect
from flask import render_template
from flask import request
from flask import url_for
from functools import wraps
from google.appengine.api import users
from google.appengine.ext import ndb
import json
import questions
import random

app = Flask(__name__)


DEFAULT_EXPERIMENT = 'default_experiment'


def experiment_key(experiment_id):
    return ndb.Key('Experiment', experiment_id)


class Participant(ndb.Model):
    completion_code = ndb.IntegerProperty()


class Response(ndb.Model):
    participant_key = ndb.KeyProperty()
    question_id = ndb.IntegerProperty()
    rules = ndb.JsonProperty()
    selected = ndb.StringProperty()
    submitted_time = ndb.DateTimeProperty(auto_now_add=True)

    @classmethod
    def has_entry(cls, exp_id, participant_key, qid):
        count = cls.query(
            cls.participant_key == participant_key,
            cls.question_id == qid,
            ancestor=experiment_key(exp_id)
        ).count(limit=1)
        return count > 0

    @classmethod
    def check_responses_complete(cls, exp_id, participant_key):
        """Returns whether the participant has completed the experiment.
        """
        count = cls.query(
            cls.participant_key == participant_key,
            ancestor=experiment_key(exp_id)
        ).count()
        return count == len(questions.DEFAULT)


@app.route('/')
def home():
    return render_template('intro.html')


@app.route('/question/<participant_key>/<int:question_id>')
def question(participant_key, question_id):
    if question_id < 0 or question_id > len(questions.DEFAULT) - 1:
        return json.dumps(questions.ERROR)  # Unknown question
    return json.dumps(questions.DEFAULT[question_id])


@app.route('/save', methods=['POST'])
def save():
    try:
        data = json.loads(request.data)
        participant_key = ndb.Key(urlsafe=data['p'])
        participant = participant_key.get()
        if participant is None:
            return json.dumps({
                'state': 'error',
                'message': 'Unknown participant.'
            });
        question_id = int(data['q'])
        if question_id < 0 or question_id > len(questions.DEFAULT) - 1:
            return json.dumps({
                'state': 'error',
                'message': 'Unknown question.'
            });
        rules = data['rules'] if 'rules' in data else ''
        selected = data['selected'] if 'selected' in data else ''

        # Don't insert duplicate responses.
        if not Response.has_entry(
                DEFAULT_EXPERIMENT, participant_key, question_id):
            response = Response(parent=experiment_key(DEFAULT_EXPERIMENT))
            response.participant_key = participant_key
            response.question_id = question_id
            response.rules = rules
            response.selected = selected
            response.put()

        if question_id == len(questions.DEFAULT) - 1:
            complete = Response.check_responses_complete(
                DEFAULT_EXPERIMENT, participant_key)
            if complete:
                code = None
                if participant.completion_code is not None:
                    code = participant.completion_code
                else:
                    code = random.randint(0, 1000000)
                    participant.completion_code = code
                    participant.put()
                return json.dumps({
                    'state': 'end',
                    'code': code
                })
            else:
                return json.dumps({
                    'state': 'error',
                    'message': 'Not all questions were completed.'
                });

        return json.dumps({
            'state': 'saved',
            'next': question_id + 1
        });
    except ValueError as e:
        print e
        return json.dumps({
            'state': 'error',
            'message': 'The wrong data was sent to the application.'
        });
    except KeyError as e:
        return json.dumps({
            'state': 'error',
            'message': 'The response was incomplete.'
        });
    except Exception as e:
        return json.dumps({
            'state': 'error',
            'message': 'An unknown error occurred.'
        });


@app.route('/study')
def start():
    participant = Participant(parent=experiment_key(DEFAULT_EXPERIMENT))
    key = participant.put()
    return render_template('webstudy.html', participantId=key.urlsafe())


@app.route('/end/<code>')
def end(code):
    return render_template('end.html', code=code)


@app.route('/error/<msg>')
def error(msg):
    return render_template('error.html', msg=msg)

@app.route('/admin')
def admin():
    user = users.get_current_user()
    if user and users.is_current_user_admin():
        query = Response.query(
            ancestor=experiment_key(DEFAULT_EXPERIMENT)
        ).order(-Response.submitted_time)
        responses = query.fetch(10)
        return render_template('admin.html', admin=True, responses=responses)
    else:
        login_url = users.create_login_url('/admin')
        return render_template('admin.html', admin=False, login_url=login_url)

