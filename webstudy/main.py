from __future__ import division

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
import math
import questions
import random

app = Flask(__name__)


DEFAULT_EXPERIMENT = 'default_experiment'


def experiment_key(experiment_id):
    return ndb.Key('Experiment', experiment_id)


class Participant(ndb.Model):
    completion_code = ndb.IntegerProperty()
    age = ndb.IntegerProperty()
    gender = ndb.StringProperty()
    ifttt_experience = ndb.StringProperty()
    programming_experience = ndb.StringProperty()
    study_comments = ndb.TextProperty()


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
            return json.dumps({
                'state': 'survey'
            })
            
        return json.dumps({
            'state': 'saved',
            'next': question_id + 1
        });
    except ValueError as e:
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


@app.route('/survey/<participant_key>')
def survey(participant_key):
    return render_template('survey.html', participant=participant_key)


@app.route('/savesurvey', methods=['POST'])
def savesurvey():
    try:
        data = json.loads(request.data)
        participant_key = ndb.Key(urlsafe=data['p'])
        participant = participant_key.get()
        if participant is None:
            return json.dumps({
                'state': 'error',
                'message': 'Unknown participant.'
            });

        complete = Response.check_responses_complete(
            DEFAULT_EXPERIMENT, participant_key)
        if complete:
            code = None
            if participant.completion_code is not None:
                code = participant.completion_code
            else:
                code = random.randint(0, 1000000)
                participant.age = int(data['age'])
                participant.gender = data['gender']
                participant.ifttt_experience = data['ifttt']
                participant.programming_experience = data['programming']
                participant.study_comments = data['otherComments']
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
    except ValueError as e:
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


@app.route('/admin')
def admin():
    user = users.get_current_user()
    if user and users.is_current_user_admin():
        return render_template('admin.html')
    else:
        login_url = users.create_login_url('/admin')
        return render_template('admin-login.html', login_url=login_url)


@app.route('/admin_api/get_all_participants/<experiment_id>')
def get_all_participants(experiment_id):
    user = users.get_current_user()
    if not user or not users.is_current_user_admin():
        return json.dumps({
            'state': 'error',
            'message': 'You must be logged in as an administrator to call this API.'
        });

    query = Participant.query(
        Participant.completion_code != None,
        ancestor=experiment_key(experiment_id)
    ).order(Participant.completion_code)
    participants = query.fetch()

    num_participants = 0
    mean_age = 0
    ages = []
    males = 0
    females = 0
    other_genders = 0
    ifttt_none = 0
    ifttt_heard_of = 0
    ifttt_used_before = 0
    programmer_no = 0
    programmer_little = 0
    programmer_yes = 0
    completion_codes = []

    for participant in participants:
        num_participants += 1
        mean_age += participant.age
        ages.append(participant.age)
        if participant.gender == 'Male':
            males += 1
        elif participant.gender == 'Female':
            females += 1
        elif participant.gender == 'Other':
            other_genders += 1
        else:
            pass

        if participant.ifttt_experience == 'No':
            ifttt_none += 1
        elif participant.ifttt_experience == 'Yes, I\'ve heard of it':
            ifttt_heard_of += 1
        elif participant.ifttt_experience == 'Yes, I\'ve used it before':
            ifttt_used_before += 1
        else:
            pass

        if participant.programming_experience == 'No':
            programmer_no += 1
        elif participant.programming_experience == 'Yes, a little':
            programmer_little += 1
        elif participant.programming_experience == 'Yes':
            programmer_yes += 1
        else:
            pass
        
        completion_codes.append(participant.completion_code)

    mean_age /= num_participants
    stddev_age = 0
    for age in ages:
        stddev_age += (age - mean_age)**2
    stddev_age = math.sqrt(stddev_age / num_participants)

    return json.dumps({
        'numParticipants': num_participants,
        'averageAge': mean_age,
        'stdDevAge': stddev_age,
        'numMales': males,
        'numFemales': females,
        'numOtherGender': other_genders,
        'malePercent': males / (males + females + other_genders),
        'femalePercent': females / (males + females + other_genders),
        'otherGenderPercent': other_genders / (males + females + other_genders),
        'iftttNos': ifttt_none,
        'iftttHeardOfs': ifttt_heard_of,
        'iftttUsedBefores': ifttt_used_before,
        'iftttNosPercent': ifttt_none / (ifttt_none + ifttt_heard_of + ifttt_used_before),
        'iftttHeardOfsPercent': ifttt_heard_of / (ifttt_none + ifttt_heard_of + ifttt_used_before),
        'iftttUsedBeforesPercent': ifttt_used_before / (ifttt_none + ifttt_heard_of + ifttt_used_before),
        'programmerNos': programmer_no,
        'programmerLittles': programmer_little,
        'programmerYes': programmer_yes,
        'programmerNosPercent': programmer_no / (programmer_no + programmer_little + programmer_yes),
        'programmerLittlesPercent': programmer_little / (programmer_no + programmer_little + programmer_yes),
        'programmerYesPercent': programmer_yes / (programmer_no + programmer_little + programmer_yes),
        'codes': completion_codes
    });
