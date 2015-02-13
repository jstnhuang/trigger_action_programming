import json

# List of questions to ask in the web study.
DEFAULT = [
    {
        'type': 'synthesis',
        'text': [
            'Suppose you want your robot to wake you up at 8:00 AM every'
            ' day.'
        ]
    },
    {
        'type': 'understanding',
        'text': [
            'What does this program do?',
        ],
        'rules': [
            {
                'id': 'unknown',
                'triggers': [
                    {
                        'name': 'doorbell_rings',
                        'params': {}
                    },
                    {
                        'name': 'daily_time',
                        'params': {
                            'hour': 15,
                            'minute': 0,
                            'beforeOrAfter': 'is'
                        }
                    }
                ],
                'actions': [
                    {
                        'name': 'switch_lights',
                        'params': {'onOrOff': 'off'}
                    }
                ]
            }
        ],
        'options': [
            'Switches the lights off at 3:00 PM',
            'Switches the lights off any time between 3:00 - 3:01 PM',
            'Switches the lights on any time other than 3:00 PM',
            'Switches the lights off at 3:00 PM and switches the lights back on at 3:01 PM'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'Suppose you work on weekdays between 9:00 AM and 5:00 PM. You'
            ' want your robot to send you an email notification whenever'
            ' someone is in the house at that time.'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'Suppose you want the lights to be on between 6:00 PM - 10:00'
            ' PM every day.'
        ]
    },
    {
        'type': 'synthesis',
        'text': [
            'Suppose you are expecting a guest to arrive at 3:00 PM. You'
            ' want your robot to open the door when the doorbell rings.'
        ]
    }
]

ERROR = {
    'type': 'error'
}
