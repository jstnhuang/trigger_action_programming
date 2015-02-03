library trigger_action_trigger_model;

import 'dart:convert';
import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';

class TimeOfDayTrigger extends Observable {
  @observable int hour = 13;
  @observable int minute = 30;
  @observable List days;
  @observable String beforeOrAfter = 'Choose before or after';
  @observable bool timeIsValid = true;
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
  
  ValidationResult validate() {
    if (!timeIsValid) {
      return new ValidationResult(false, message: 'Time of day: invalid time.');
    }
    if (beforeOrAfter != 'is' && beforeOrAfter != 'is before' && beforeOrAfter != 'is after') {
      return new ValidationResult(false, message: 'Time of day: choose "is", "is before," or "is after."');
    }
    if (days.isEmpty) {
      return new ValidationResult(false, message: 'Time of day: pick at least one day of the week.');
    }
    return new ValidationResult(true);
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