import 'package:polymer/polymer.dart';
import 'dart:async';
import 'dart:convert';
import 'dart:html';
import 'dart:js';
import 'roslibjs.dart';

@CustomTag('statement-card')
class StatementCard extends PolymerElement {
  @published String id = 'unknown';
  @published String trigger_name = '';
  @published Map<String, String> trigger_params = {};
  @published String action_name = '';
  @published Map<String, String> action_params = {};
  @published bool isNew = false;
  
  Ros _ros;
  Service _addStatementClient;
  Service _updateStatementClient;
  Service _deleteStatementClient;
  
  Element _saveButton;
  Element _deleteButton;
  
  StatementCard.created() : super.created() {
    this._ros = new Ros(robotWebsocketUrl);
    this._addStatementClient = new Service(this._ros, '/add_statement', 'trigger_action_programming/AddStatement');
    this._updateStatementClient = new Service(this._ros, '/update_statement', 'trigger_action_programming/UpdateStatement');
    this._deleteStatementClient = new Service(this._ros, '/delete_statement', 'trigger_action_programming/DeleteStatement');
    this._saveButton = this.$['#saveButton'];
    this._deleteButton = this.$['#deleteButton'];
  }
  
  void save(Event event, Object detail, Element sender) {
    var statementObject = {
      'id': id,
      'trigger_name': trigger_name,
      'trigger_params': JSON.encode(trigger_params),
      'action_name': action_name,
      'action_params': JSON.encode(action_params)
    };
    if (isNew) {
      var request = new ServiceRequest({'statement': statementObject});
      Future future = this._addStatementClient.call(request);
      future.then((JsObject results) {
        id = results['id'];
        isNew = false;
      });
    } else {
      var request = new ServiceRequest(
        {
          'id': id,
          'updated_statement': statementObject
        }
      );
      Future future = this._updateStatementClient.call(request);
    }
  }
  
  void delete(Event event, Object detail, Element sender) {
    if (isNew) {
      this.remove();
    } else {
      var request = new ServiceRequest({'id': id});
      Future future = this._deleteStatementClient.call(request);
      future.then((JsObject results) { this.remove(); });
    }
  }
}

