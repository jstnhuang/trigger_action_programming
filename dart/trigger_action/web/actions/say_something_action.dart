import 'package:polymer/polymer.dart';

@CustomTag('say-something-action-options')
class SaySomethingActionOptions extends PolymerElement {
  @published String speech = "";
  
  SaySomethingActionOptions.created() : super.created() {
  }
}
