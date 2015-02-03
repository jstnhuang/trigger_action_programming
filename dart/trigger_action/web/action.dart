import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';
import 'dart:html';

@CustomTag('action-selector')
class ActionSelectorElement extends PolymerElement {
  @published Action model;
  @published bool opened = false;
  @published bool readOnly = false;
  
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
    model.name = sender.attributes['data-name'];
    model.currentAction = actionFactory(model.name, model.params);
  }
}