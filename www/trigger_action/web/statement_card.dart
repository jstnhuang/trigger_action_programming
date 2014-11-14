import 'package:polymer/polymer.dart';

@CustomTag('statement-card')
class StatementCard extends PolymerElement {
  @published String trigger_name = "";
  @published Map<String, String> trigger_params = {};
  @published String action_name = "";
  @published Map<String, String> action_params = {};
  
  StatementCard.created() : super.created() {
  }
}

