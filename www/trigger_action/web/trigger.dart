import 'package:polymer/polymer.dart';
import 'dart:html';

@CustomTag('trigger-selector')
class TriggerSelectorElement extends PolymerElement {
  @published String name;
  @published Map<String, String> params;
  @published bool opened = false;
  
  Map<String, String> display_names = {
    'person_detected': 'Person detected',
  };
  
  TriggerSelectorElement.created() : super.created() {
  }
  
  void toggle() {
    opened = !opened;
  }
  
  void triggerSelected(Event event, Object detail, Element sender) {
    toggle();
    name = sender.attributes['data-name'];
  }
}