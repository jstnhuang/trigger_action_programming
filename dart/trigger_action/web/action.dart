import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';
import 'dart:async';
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
    // TODO: Not sure if this is the right way to wait for the model to be ready.
    Future future = new Future(() {
      while (model == null) {
      }
    });
    future.timeout(new Duration(milliseconds: 500));
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