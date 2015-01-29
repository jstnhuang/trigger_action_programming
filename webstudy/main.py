from flask import Flask
from flask import make_response
from flask import redirect
from flask import render_template
from flask import request
from flask import url_for
from functools import wraps
import json
import questions
import random

app = Flask(__name__)


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
    participant_id = random.randint(0, 100000)
    return redirect('/txg/webstudy.html?p={}&q=0'.format(participant_id))


@app.route('/question/<int:participant_id>/<int:question_id>')
def question(participant_id, question_id):
    response = make_response(json.dumps(questions.SYNTHESIS[question_id]))
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
        rules = data['rules']
        if question_id == len(questions.SYNTHESIS) - 1:
            # TODO: Check if all questions have actually been answered. If not,
            # redirect to an error page.
            return '/txg/webstudy-end.html'
        question_id += 1
        # TODO: Save data to datastore
        print rules
        return '/txg/webstudy.html?p={}&q={}'.format(
            participant_id, question_id)
    except ValueError:
        # TODO: redirect to an error page
        return '/'
