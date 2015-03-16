(function (document) {
  'use strict';

  var rules = [
    {
      triggers: [
        {
          key: 'motion_detector',
          displayName: 'Motion detector',
          params: { }
        }
      ],
      action: {
        key: 'set_door_lock',
        displayName: 'Set door lock',
        params: {
          lockOption: 'timed_unlock',
          seconds: 10,
        }
      },
      name: 'If motion is detected, then briefly unlock the door.'
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
