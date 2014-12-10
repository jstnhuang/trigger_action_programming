import 'dart:html';
import 'package:polymer/polymer.dart';
import 'trigger.dart';

@CustomTag('trigger-list')
class TriggerList extends PolymerElement {
  List<TriggerModel> triggers = toObservable([]);

  TriggerList.created() : super.created() {
  }

  void attached() {
    if (triggers.length == 0) {
      TriggerModel trigger = new TriggerModel('', {}, true);
      triggers.add(trigger);
    }
  }

  void createTrigger(MouseEvent event) {
    TriggerModel trigger = new TriggerModel('', {}, false);
    triggers.add(trigger);
  }
}
