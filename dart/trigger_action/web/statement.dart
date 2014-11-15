import "dart:js";
import "dart:convert";

class Statement {
  String id;
  String trigger_name;
  Map<String, String> trigger_params;
  String action_name;
  Map<String, String> action_params;
  Statement(this.id, this.trigger_name, this.trigger_params, this.action_name, this.action_params);
  Statement.fromJs(JsObject jsStatement)
    : id = jsStatement['id'],
      trigger_name = jsStatement['trigger_name'],
      trigger_params = JSON.decode(jsStatement['trigger_params']),
      action_name = jsStatement['action_name'],
      action_params =  JSON.decode(jsStatement['action_params']) {
  }
}