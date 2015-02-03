library trigger_action_model;
import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';

class SaySomethingAction extends Observable {
  @observable String speech = '';
  @observable bool readOnly = false;
  SaySomethingAction(this.speech);
  
  ValidationResult validate() {
    return new ValidationResult(speech != '', message: 'Say something: text can\'t be empty.');
  }
}