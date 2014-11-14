import 'package:polymer/polymer.dart';

@CustomTag('action-element')
class ActionElement extends PolymerElement {
  @published String name;
  @published Map<String, String> params;
  @published bool opened = false;
  
  Map<String, String> display_names = {
    'say_something': 'Say something'
  };
  
  ActionElement.created() : super.created() {
  }
  
  void toggle() {
    opened = !opened;
  }
}