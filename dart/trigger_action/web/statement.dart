import 'package:polymer/polymer.dart';
import 'dart:async';
import 'dart:convert';
import 'dart:html';
import 'dart:js';
import 'roslibjs.dart';
import 'trigger.dart';

class StatementModel {
  String id;
  List<TriggerModel> triggers = [];
  String action_name;
  Map<String, String> action_params;
  bool isNew = false;
  StatementModel(this.id, this.triggers, this.action_name, this.action_params, this.isNew);
  StatementModel.fromJs(JsObject jsStatement)
    : id = jsStatement['id'],
      triggers = [],
//      triggers = JSON.decode(jsStatement['triggers'], reviver: (key, value) {
//        return TriggerModel.fromJs(jsStatement['triggers'][key]);
//      }),
      action_name = jsStatement['action_name'],
      action_params =  JSON.decode(jsStatement['action_params']) {
  }
}

@CustomTag('statement-card')
class StatementCard extends PolymerElement {
  @published String id = 'unknown';
  List<TriggerModel> triggers = toObservable([]);
  @published String action_name = '';
  @published Map<String, String> action_params = {};
  @published bool isNew = false;
  @published String saveLabel = '';
  @published String deleteLabel = '';
  @published List<StatementModel> parentList;
  @published bool opened = false;
  @published bool saveDisabled = true;

  @published Ros ros;
  Service _addStatementClient;
  Service _updateStatementClient;
  Service _deleteStatementClient;

  StatementCard.created() : super.created() {
    updateButtons();
  }

  void attached() {
    this._addStatementClient = new Service(this.ros, '/add_statement', 'trigger_action_programming/AddStatement');
    this._updateStatementClient = new Service(this.ros, '/update_statement', 'trigger_action_programming/UpdateStatement');
    this._deleteStatementClient = new Service(this.ros, '/delete_statement', 'trigger_action_programming/DeleteStatement');
    updateButtons();
    Timer timer = new Timer(new Duration(milliseconds: 50), () {
      opened = true;
    });
    if (triggers.length == 0) {
      TriggerModel trigger = new TriggerModel('', {}, true);
      triggers.add(trigger);
      print(triggers.length);
    }
  }

  void toast(String text) {
    Element t = document.querySelector('body /deep/ #toast');
    t.attributes['text'] = text;
    t.show();
  }

  void updateButtons() {
    if (isNew) {
      saveLabel = 'Add rule';
      deleteLabel = 'Cancel';
    } else {
      saveLabel = 'Update rule';
      deleteLabel = 'Delete';
    }
  }
  
  void createTrigger(MouseEvent event) {
    TriggerModel trigger = new TriggerModel('', {}, false);
    triggers.add(trigger);
  }

  void save(Event event, Object detail, Element sender) {
    var statementObject = {
      'id': id,
      'triggers': JSON.encode(triggers),
      'action_name': action_name,
      'action_params': JSON.encode(action_params)
    };
    print(statementObject);
    if (isNew) {
      var request = new ServiceRequest({'statement': statementObject});
      Future future = this._addStatementClient.call(request);
      future.then((JsObject results) {
        id = results['id'];
        isNew = false;
        updateButtons();
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
      opened=false;
      Timer timer = new Timer(new Duration(milliseconds: 333), () {
        this.remove();
      });
    } else {
      var request = new ServiceRequest({'id': id});
      Future future = this._deleteStatementClient.call(request);
      future.then((JsObject results) {
        toast('Deleted rule.');
        opened=false;
        Timer timer = new Timer(new Duration(milliseconds: 333), () {
          parentList.removeWhere((StatementModel card) {
            return card.id == this.id;
          });
        });
      });
    }
  }
}

