import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';
import 'roslibjs.dart';

@CustomTag('trigger-action-app')
class TriggerActionAppElement extends PolymerElement {
  Ros ros;
  @observable bool isConnected;
  StatementList statementList;

  TriggerActionAppElement.created() : super.created() {
    ros = new Ros(localWebsocketUrl);
    statementList = new StatementList(ros);
  }

  void attached() {
    ros.connect().then((Event event) {
        isConnected = true;
        print('Connected to websocket server.');
      })
    .catchError((Event error) {
      isConnected = false;
      print('Connection error: $error');
    });
  }
}
