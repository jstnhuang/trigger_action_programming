import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('trigger-action-app')
class TriggerActionAppElement extends PolymerElement {
  @published TriggerActionApp model;
  @observable bool isConnected;
  @published String type;

  TriggerActionAppElement.created() : super.created() {
    model = new TriggerActionApp(type);
    model.ruleDb.connect().then((Event event) {
      model.isConnected = true;
      print('Connected to websocket server.');
    })
    .catchError((Event error) {
      model.isConnected = false;
      print('Connection error: $error');
    });
  }

  void attached() {

  }
}
