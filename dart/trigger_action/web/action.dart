import 'package:polymer/polymer.dart';
import 'dart:html';

@CustomTag('action-selector')
class ActionSelectorElement extends PolymerElement {
  @published String name;
  @published Map<String, String> params;
  @published bool opened = false;
  
  Map<String, String> display_names = {
    'say_something': 'Say something'
  };
  
  ActionSelectorElement.created() : super.created() {
  }
  
  void toggle() {
    opened = !opened;
  }
  
  void actionSelected(Event event, Object detail, Element sender) {
    toggle();
    name = sender.attributes['data-name'];
  }
}