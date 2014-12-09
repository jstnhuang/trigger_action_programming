import 'dart:async';
import 'dart:convert';
import 'dart:html';
import 'dart:js';
import 'package:polymer/polymer.dart';

class TriggerModel extends Observable {
  @observable String name;
  @observable Map<String, String> params;
  @observable bool isFirst = false;
  TriggerModel(this.name, this.params, this.isFirst);
  TriggerModel.fromJs(JsObject obj)
    : name = obj['name'],
      params = JSON.decode(obj['params']) {
  }
  toJson() {
    return {
      'name': name,
      'params': JSON.encode(params)
    };
  }
}

@CustomTag('trigger-selector')
class TriggerSelectorElement extends PolymerElement {
  @published String name;
  @published Map<String, String> params;
  @published bool isFirst = false;
  @published bool opened = false;
  @published List<TriggerModel> parentList;
  
  Map<String, String> display_names = {
    'person_detected': 'Person detected',
    'time_of_day': 'Time of day'
  };
  
  TriggerSelectorElement.created() : super.created() {
    if (isFirst) {
      opened = true;
    } else {
      new Timer(new Duration(milliseconds: 50), toggle);
    }
  }
  
  void attached() {
  }
  
  void toggle() {
    opened = !opened;
  }
  
  void triggerSelected(Event event, Object detail, Element sender) {
    toggle();
    name = sender.attributes['data-name'];
  }
}