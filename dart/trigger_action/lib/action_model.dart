library trigger_action_action_model;

import 'dart:convert';
import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';

class SaySomethingAction extends Observable {
  @observable String speech = '';
  @observable bool readOnly = false;
  SaySomethingAction(this.speech);
  
  ValidationResult validate() {
    return new ValidationResult(speech != '', 'Say something: text can\'t be empty.');
  }
  
  Object toJson() {
    return {
      'name': 'say_something',
      'params': JSON.encode({
        'speech': speech,
      })
    };
  }
}