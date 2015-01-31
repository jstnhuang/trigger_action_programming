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


class Response(ndb.Model):
    participant_id = ndb.IntegerProperty()
    question_id = ndb.IntegerProperty()
    rules = ndb.JsonProperty()
    selected = ndb.StringProperty()


def allow_localhost(f):
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


@app.route('/question/<int:participant_id>/<int:question_id>')
def question(participant_id, question_id):
    response = make_response(json.dumps(questions.DEFAULT[question_id]))
    response.headers['Access-Control-Allow-Origin'] = (
        'http://localhost:8080')
    return response


@app.route('/next', methods=['POST'])
@allow_localhost
def next():
    try:
        data = json.loads(request.data)
        participant_id = int(data['p'])
        question_id = int(data['q'])
        rules = data['rules'] if 'rules' in data else ''
        selected = data['selected'] if 'selected' in data else ''
        response = Response(parent=experiment_key(DEFAULT_EXPERIMENT))
        response.participant_id = participant_id
        response.question_id = question_id
        response.rules = rules
        response.selected = selected
        response.put()

        if question_id == len(questions.DEFAULT) - 1:
            # TODO: Check if all questions have actually been answered. If not,
            # redirect to an error page.
            return url_for('end')

        question_id += 1
        return '/txg/webstudy.html?p={}&q={}'.format(
            participant_id, question_id)
    except ValueError:
        return url_for('error')


@app.route('/start')
def start():
    participant_id = random.randint(0, 100000)
    return redirect('/txg/webstudy.html?p={}&q=0'.format(participant_id))


@app.route('/end')
def end():
    return render_template('end.html')


@app.route('/error')
def error():
    return render_template('error.html')
