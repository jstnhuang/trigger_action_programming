import 'dart:html';
import 'package:polymer/polymer.dart';
import 'trigger.dart';

@CustomTag('trigger-list')
class TriggerList extends PolymerElement {
  List<TriggerModel> triggers = toObservable([]);

  TriggerList.created() : super.created() {
  }

  void attached() {
    print(triggers.length);
    if (triggers.length == 0) {
      TriggerModel trigger = new TriggerModel('', {}, true);
      triggers.add(trigger);
      print(triggers.length);
    }
  }

  void createTrigger(MouseEvent event) {
    print('created trigger');
    TriggerModel trigger = new TriggerModel('', {}, false);
    triggers.add(trigger);
    print(triggers.length);
  }
}
