import json

# List of questions to ask in the web study.
DEFAULT = [
    {
        'type': 'synthesis',
        'text': [
            'You want the lights to turn on at 6:00 PM every day.'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'You want to be notified, via email, should a person be detected'
            ' in the house while everyone is at work (between 9:00 AM and'
            ' 5:00 PM every day).',
            'The email can say anything you want.'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'Your work starts at 9:00 am.',
            'On days when you get to work on time, you want to send an email'
            ' to yourself saying, "I got to work on time!"'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'You want the thermostat to be off as much as possible, unless the'
            ' temperature outside is below 40 degrees, in which case the'
            ' thermostat should be set to 72 degrees.'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'You want a pot of coffee to be brewed when it\'s below 40 degrees'
            ' outside, but only before 10:00 AM every day.'
        ]
    },
    {
        'type': 'understanding',
        'text': [
            'If this is the only rule, do you expect the lights to turn off,'
            ' and if so, when?'
        ],
        'rules': [
            {
                'id': 'unknown',
                'mode': 'summary',
                'triggers': [
                    {
                        'key': 'daily_time',
                        'displayName': 'Daily time',
                        'params': {
                            'startHour': 18,
                            'startMinute': 0,
                            'isOrBetween': 'is',
                            'endHour': 19,
                            'endMinute': 0,
                        }
                    }
                ],
                'action': {
                    'key': 'switch_lights',
                    'displayName': 'Switch lights',
                    'params': {'onOrOff': 'on'}
                },
                'name': 'If the time is 06:00 PM, then switch the lights on'
            }
        ],
        'options': [
            'Yes, the lights will turn off at 6:01 PM',
            'Yes, the lights will turn off at 7:00 PM',
            'Yes, the lights will turn off at midnight',
            'No, the lights will not turn off'
        ]
    },
    {
        'type': 'understanding',
        'text': [
            'If this is the only rule, and you arrive home at 5:00 PM, do you'
            ' expect the lights to turn on, and if so, when?'
        ],
        'rules': [
            {
                'id': 'unknown',
                'mode': 'summary',
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
                        'key': 'daily_time',
                        'displayName': 'Daily time',
                        'params': {
                            'endHour': 23,
                            'endMinute': 0,
                            'isOrBetween': 'is between',
                            'startHour': 18,
                            'startMinute': 0
                        }
                    }
                ],
                'action': {
                    'key': 'switch_lights',
                    'displayName': 'Switch lights',
                    'params': {'onOrOff': 'on'}
                },
                'name': (
                    'If I arrive at home and the time is between 06:00 PM'
                    ' and 11:00 PM, then switch the lights on'
                )
            }
        ],
        'options': [
            'Yes, the lights will turn on at 6:00 PM',
            'Yes, the lights will turn on any time between 6:00 PM and 11:00'
            ' PM',
            'Yes, the lights will turn on at 11:00 PM',
            'No, the lights will not turn on'
        ]
    },
    {
        'type': 'understanding',
        'text': [
            'You are expecting visitors to your house at 3:00 PM. If you'
            ' wanted them to be let in automatically, would you use this rule?'
            ' If so, when do you expect the door will unlock?'
        ],
        'rules': [
            {
                'id': 'unknown',
                'mode': 'summary',
                'triggers': [
                    {
                        'key': 'doorbell',
                        'displayName': 'Doorbell rings',
                        'params': { }
                    },
                    {
                        'key': 'daily_time',
                        'displayName': 'Daily time',
                        'params': {
                            'endHour': 23,
                            'endMinute': 0,
                            'isOrBetween': 'is',
                            'startHour': 15,
                            'startMinute': 0
                        }
                    }
                ],
                'action': {
                    'key': 'set_door_lock',
                    'displayName': 'Set door lock',
                    'params': {
                        'lockOption': 'timed_unlock',
                        'seconds': 10
                    }
                },
                'name': (
                    'If the doorbell rings and the time is 03:00 PM, then'
                    ' briefly unlock the door'
                )
            }
        ],
        'options': [
            'Yes, I would. The door will unlock if the visitors ring the'
            ' doorbell at exactly 3:00 PM',
            'Yes, I would. The door will unlock if the visitors ring the'
            ' doorbell between 3:00 - 3:01 PM',
            'Yes, I would. The door will unlock if the visitors ring the'
            ' doorbell between 3:00 - 4:00 PM',
            'No, I would not use a rule like this'
        ]
    },
    {
        'type': 'understanding',
        'text': [
            'When will the thermostat be set to 75 degrees and when will it'
            ' turn off?'
        ],
        'rules': [
            {
                'id': 'unknown',
                'mode': 'summary',
                'triggers': [
                    {
                        'key': 'weather',
                        'displayName': 'Weather',
                        'params': {
                            'condition': 'snow',
                            'isAboveBelow': 'Choose is/is above/is below',
                            'temperature': 70
                        }
                    }
                ],
                'action': {
                    'key': 'set_thermostat',
                    'displayName': 'Set thermostat',
                    'params': {
                        'onOrOff': 'on',
                        'temperature': 75
                    }
                },
                'name': 'If it is snowing, then set thermostat to 75 degrees'
            }
        ],
        'options': [
            'It will turn on as soon as it starts snowing, it won\'t turn off',
            'It will turn on as soon as it starts snowing, it turns off as'
            ' soon as it stops snowing',
            'It will turn on any time while it\'s snowing, it won\'t turn off',
            'It will turn on any time while it\'s snowing, it will turn off as'
            ' soon as it stops snowing'
        ]
    },
    {
        'type': 'understanding',
        'text': [
            'Suppose it was cold all night. Do you expect the coffee brewer to'
            ' start, and if so, when?'
        ],
        'rules': [
            {
                'id': 'unknown',
                'mode': 'summary',
                'triggers': [
                    {
                        'key': 'daily_time',
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
                        'key': 'weather',
                        'displayName': 'Weather',
                        'params': {
                            'condition': 'temperature',
                            'isAboveBelow': 'is below',
                            'temperature': 40
                        }
                    }
                ],
                'action': {
                    'key': 'brew_coffee',
                    'displayName': 'Brew coffee',
                    'params': { }
                },
                'name': (
                    'If the time is between 07:00 AM and 10:00 AM and the'
                    ' temperature outside is below 40 degrees, then brew a pot'
                    ' of coffee'
                )
            }
        ],
        'options': [
            'Yes, it will start at 7:00 AM',
            'Yes, it will start at 10:00 AM',
            'Yes, it will start any time between 7:00 - 10:00 AM',
            'No, it will not start'
        ]
    },
]

ERROR = {
    'type': 'error'
}
