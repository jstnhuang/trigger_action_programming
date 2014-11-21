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
  @published String saveLabel = '';

  @published Ros ros;
  Service _addStatementClient;
  Service _updateStatementClient;
  Service _deleteStatementClient;

  StatementCard.created() : super.created() {
    updateSaveButton();
  }

  void attached() {
    this._addStatementClient = new Service(this.ros, '/add_statement', 'trigger_action_programming/AddStatement');
    this._updateStatementClient = new Service(this.ros, '/update_statement', 'trigger_action_programming/UpdateStatement');
    this._deleteStatementClient = new Service(this.ros, '/delete_statement', 'trigger_action_programming/DeleteStatement');
    updateSaveButton();
  }

  void toast(String text) {
    Element t = document.querySelector('body /deep/ #toast');
    t.attributes['text'] = text;
    t.show();
  }

  void updateSaveButton() {
    if (isNew) {
      saveLabel = 'Add rule';
    } else {
      saveLabel = 'Update rule';
    }
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
        saveLabel = 'Update rule';
        toast('Added new rule.');
      });
    } else {
      var request = new ServiceRequest(
        {
          'id': id,
          'updated_statement': statementObject
        }
      );
      Future future = this._updateStatementClient.call(request);
      future.then((JsObject results) {
        toast('Updated rule.');
      });
    }
  }

  void delete(Event event, Object detail, Element sender) {
    if (isNew) {
      this.remove();
    } else {
      var request = new ServiceRequest({'id': id});
      Future future = this._deleteStatementClient.call(request);
      future.then((JsObject results) {
        toast('Deleted rule.');
        this.remove();
      });
    }
  }
}

