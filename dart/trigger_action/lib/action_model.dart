library trigger_action_model;
import 'package:polymer/polymer.dart';

class SaySomethingAction extends Observable {
  @observable String speech = "";
  @observable bool readOnly = false;
  SaySomethingAction(this.speech);
  
  bool isValid() {
    return speech != "";
  }
}