import 'package:polymer/polymer.dart';

@CustomTag('trigger-selector')
class TriggerSelectorElement extends PolymerElement {
  @published String name;
  @published Map<String, String> params;
  @published bool opened = false;
  
  Map<String, String> display_names = {
    'person_detected': 'Person detected',
    'time': 'Time is'
  };
  
  TriggerSelectorElement.created() : super.created() {
  }
  
  void toggle() {
    opened = !opened;
  }
}