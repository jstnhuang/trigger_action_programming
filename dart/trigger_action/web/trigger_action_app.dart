import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('trigger-action-app')
class TriggerActionAppElement extends PolymerElement {
  @observable TriggerActionApp model;
  @observable bool isConnected;
  @published String websocketUrl;
  @published bool webstudy = false; // Whether this is part of webstudy or not.

  TriggerActionAppElement.created() : super.created() {
    model = new TriggerActionApp(websocketUrl, webstudy);
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
