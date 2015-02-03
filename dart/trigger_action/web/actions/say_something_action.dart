import 'package:polymer/polymer.dart';
import 'package:trigger_action/action_model.dart';

@CustomTag('say-something-action-options')
class SaySomethingActionOptions extends PolymerElement {
  @published SaySomethingAction model;
  @published bool readOnly = false;
  
  SaySomethingActionOptions.created() : super.created() {
    
  }
}
