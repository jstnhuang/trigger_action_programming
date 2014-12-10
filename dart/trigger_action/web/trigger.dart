import 'dart:async';
import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('trigger-selector')
class TriggerSelectorElement extends PolymerElement {
  @published Trigger model;
  @observable bool opened = false;
  @published bool isFirst = false;
  @published bool isOnly = false;
  @observable bool animationStart = false;
  
  Map<String, String> display_names = {
    'person_detected': 'Person detected',
    'time_of_day': 'Time of day'
  };
  
  TriggerSelectorElement.created() : super.created() {
  }
  
  void attached() {
    opened = model.name == '';
    if (isFirst) {
      shadowRoot.querySelector('#animation').style.display = 'inline';
    }
    new Timer(new Duration(milliseconds: 50), () {
      animationStart = true;
    });
    // core-overlay changes itself to overflow: hidden, so we flip it back
    // to visible once it's done.
    new Timer(new Duration(milliseconds: 250), () {
      shadowRoot.querySelector('#animation').style.overflow = 'visible';
    });
  }
  
  void toggle() {
    opened = !opened;
  }
  
  void triggerSelected(Event event, Object detail, Element sender) {
    toggle();
    model.name = sender.attributes['data-name'];
  }
  
  void delete() {
    dispatchEvent(new CustomEvent(
      'delete-trigger',
      detail: {'element': this}
    ));
  }
}