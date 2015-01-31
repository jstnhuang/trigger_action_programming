from flask import Flask
from flask import make_response
from flask import redirect
from flask import render_template
from flask import request
from flask import url_for
from functools import wraps
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


def allow_localhost(f):
    """Allows cross-origin requests from localhost:8080.

    Used for debugging the Dart app, which pub serves from that origin.
    """
    @wraps(f)
    def decorated_function(*args, **kwargs):
        response = make_response(f(*args, **kwargs))
        response.headers['Access-Control-Allow-Origin'] = (
            'http://localhost:8080')
        return response
    return decorated_function


@app.route('/')
def home():
    return render_template('start.html')


@app.route('/question/<participant_key>/<int:question_id>')
@allow_localhost
def question(participant_key, question_id):
    if question_id < 0 or question_id > len(questions.DEFAULT) - 1:
        return redirect(url_for('error'))  # Unknown question
    return json.dumps(questions.DEFAULT[question_id])


@app.route('/next', methods=['POST'])
@allow_localhost
def next():
    try:
        data = json.loads(request.data)
        participant_key = ndb.Key(urlsafe=data['p'])
        participant = participant_key.get()
        if participant is None:
            return url_for('error')  # Unknown participant
        question_id = int(data['q'])
        if question_id < 0 or question_id > len(questions.DEFAULT) - 1:
            return url_for('error')  # Unknown question
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
                return url_for('end', code=code)
            else:
                return url_for('error')

        return '/txg/webstudy.html?p={}&q={}'.format(
            participant_key.urlsafe(), question_id + 1)
    except ValueError:
        return url_for(
            'error', msg='The wrong data was sent to the application.')
    except KeyError:
        return url_for('error', msg='The response was incomplete.')
    except Exception as e:
        return url_for('error', msg='An unknown error occurred.')


@app.route('/start')
def start():
    participant = Participant(parent=experiment_key(DEFAULT_EXPERIMENT))
    key = participant.put()
    return redirect('/txg/webstudy.html?p={}&q=0'.format(key.urlsafe()))


@app.route('/end/<code>')
def end(code):
    return render_template('end.html', code=code)


@app.route('/error/<msg>')
def error(msg):
    return render_template('error.html', msg=msg)
