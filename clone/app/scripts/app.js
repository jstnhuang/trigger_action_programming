(function (document) {
  'use strict';

  var rules = [
    {
      triggers: [
        {
          key: 'my_location',
          displayName: 'My location',
          params: {
            verb: 'am currently at',
            location: 'home'
          }
        }
      ],
      action: {
        key: 'set_thermostat',
        displayName: 'Set thermostat',
        params: {
          onOrOff: 'on',
          temperature: 72,
        }
      },
      name: 'If I am currently at home, then set the thermostat to 72 degrees'
    },
    {
      triggers: [
        {
          key: 'daily_time',
          displayName: 'Daily time',
          params: {
            endHour: 17,
            endMinute: 0,
            isOrBetween: 'is',
            startHour: 8,
            startMinute: 0
          }
        }
      ],
      action: {
        key: 'send_email',
        displayName: 'Send email',
        params: {
          content: 'Test sentence 1. Test sentence 2.'
        }
      },
      name: 'If the time is 08:00 AM, then send myself an email'
    }              
  ];

  document.addEventListener('polymer-ready', function () {
    var app = document.querySelector('trigger-action-app');
    app.rules = rules;
  });

// wrap document so it plays nice with other libraries
// http://www.polymer-project.org/platform/shadow-dom.html#wrappers
})(wrap(document));
