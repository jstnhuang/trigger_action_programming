import 'dart:async';
import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('trigger-selector')
class TriggerSelectorElement extends PolymerElement {
  @published Trigger model;
  @observable bool opened = false;
  @observable bool animationStart = false;
  
  Map<String, String> display_names = {
    'person_detected': 'Person detected',
    'time_of_day': 'Time of day'
  };
  
  TriggerSelectorElement.created() : super.created() {
  }
  
  void attached() {
    opened = model.name == '';
    if (model.isFirst) {
      shadowRoot.querySelector('#animation').setAttribute('style', 'display: inline');
    }
    new Timer(new Duration(milliseconds: 50), () {animationStart = true;});
  }
  
  void toggle() {
    opened = !opened;
  }
  
  void triggerSelected(Event event, Object detail, Element sender) {
    toggle();
    model.name = sender.attributes['data-name'];
  }
}