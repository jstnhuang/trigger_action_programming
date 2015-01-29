from flask import Flask
from flask import make_response
from flask import redirect
from flask import render_template
from flask import url_for
import json
import questions
import random

app = Flask(__name__)


@app.route('/')
def home():
    participant_id = random.randint(0, 100000)
    return redirect('/txg/webstudy.html?p={}&q=1'.format(participant_id))


@app.route('/question/<int:participant_id>/<int:question_id>')
def question(participant_id, question_id):
    response = make_response(json.dumps(questions.SYNTHESIS[question_id]))
    response.headers['Access-Control-Allow-Origin'] = 'http://localhost:8080'
    return response
