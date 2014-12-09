import "package:polymer/polymer.dart";
import "roslibjs.dart";
import "dart:html";

@CustomTag('trigger-action-app')
class TriggerActionAppElement extends PolymerElement {
  @published Ros ros;
  @published bool isConnected;

  TriggerActionAppElement.created() : super.created() {
    ros = new Ros(localWebsocketUrl);
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
