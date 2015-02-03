library trigger_action_trigger_model;

import 'dart:convert';
import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';

class TimeOfDayTrigger extends Observable {
  @observable int hour = 13;
  @observable int minute = 30;
  @observable List days;
  @observable String beforeOrAfter = 'Choose before or after';
  TimeOfDayTrigger(this.hour, this.minute, this.days, this.beforeOrAfter) {
    if (hour == null) {
      this.hour = 9;
    }
    if (minute == null) {
      this.minute = 30;
    }
    if (days == null) {
      this.days = toObservable([]);
    }
  }
  
  toJson() {
    return {
      'name': 'time_of_day',
      'params': JSON.encode({
        'hour': hour,
        'minute': minute,
        'days': days,
        'before_or_after': beforeOrAfter
      })
    };
  }
}

class PersonDetectedTrigger extends Observable {
  // Currently blank because it has no options.
  toJson() {
    return {
      'name': 'person_detected',
      'params': JSON.encode({})
    };
  }
  
  ValidationResult validate() {
    return new ValidationResult(true);
  }
}