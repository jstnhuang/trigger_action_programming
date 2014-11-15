import 'package:polymer/polymer.dart';

@CustomTag('say-something-action-options')
class StatementCard extends PolymerElement {
  @published String speech = "";
  
  StatementCard.created() : super.created() {
  }
}
