import 'dart:async';
import 'dart:html';
import 'package:polymer/polymer.dart';
import 'package:trigger_action/model.dart';

@CustomTag('trigger-selector')
class TriggerSelectorElement extends PolymerElement {
  @published Trigger model;
  @observable bool opened = false;
  @published bool isFirst = false;
  @published bool isOnly = false;
  @published bool readOnly = false;
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
  }
  
  void toggle() {
    opened = !opened;
  }
  
  void triggerSelected(Event event, Object detail, Element sender) {
    toggle();
    model.name = sender.attributes['data-name'];
    model.currentTrigger = triggerFactory(model.name, model.params);
  }
  
  void delete() {
    dispatchEvent(new CustomEvent(
      'delete-trigger',
      detail: {'element': this}
    ));
  }
}