(function (document) {
  'use strict';

  var rules = [
    {
      triggers: [
        {
          name: 'weekly_time',
          displayName: 'Weekly time',
          params: {
            startHour: 15,
            startMinute: 0,
            sunday: false,
            monday: false,
            tuesday: false,
            wednesday: true,
            thursday: false,
            friday: false,
            saturday: false
          }
        }
      ],
      action: {
        name: 'say_something',
        displayName: 'Say something',
        params: {
          content: 'Time for the lab meeting!',
        }
      },
      name: 'If the time is 03:00 PM on Wednesdays, then say something.'
    },
  ];

  document.addEventListener('polymer-ready', function () {
    var app = document.querySelector('trigger-action-app');
    app.rules = rules;
    app.triggerSet = ['weekly_time', 'weather', 'doorbell', 'my_location', 
      'motion_detector'];
    app.actionSet = ['brew_coffee', 'send_email', 'set_door_lock', 'switch_lights',
      'set_thermostat'];
  });

// wrap document so it plays nice with other libraries
// http://www.polymer-project.org/platform/shadow-dom.html#wrappers
})(wrap(document));
