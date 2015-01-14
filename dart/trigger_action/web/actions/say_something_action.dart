import 'package:polymer/polymer.dart';

@CustomTag('say-something-action-options')
class SaySomethingActionOptions extends PolymerElement {
  @published String speech = "";
  @published bool readOnly = false;
  
  SaySomethingActionOptions.created() : super.created() {
  }
}
