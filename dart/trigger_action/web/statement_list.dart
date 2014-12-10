import 'dart:async';
import 'dart:convert';
import 'dart:html';
import 'dart:js';
import 'model.dart';
import 'package:polymer/polymer.dart';
import 'roslibjs.dart';

@CustomTag('statement-list')
class StatementListElement extends PolymerElement {
  @published StatementList model;
  Service _getStatementsClient;
  Service _addStatementClient;
  Service _updateStatementClient;
  Service _deleteStatementClient;

  StatementListElement.created() : super.created() {
  }

  void attached() {
    _getStatementsClient = new Service(model.ros, '/get_all_rules', 'trigger_action_programming/GetAllRules');
    _addStatementClient = new Service(model.ros, '/add_rule', 'trigger_action_programming/AddRule');
    _updateStatementClient = new Service(model.ros, '/update_rule', 'trigger_action_programming/UpdateRule');
    _deleteStatementClient = new Service(model.ros, '/delete_rule', 'trigger_action_programming/DeleteRule');

    var request = new ServiceRequest({});
    _getStatementsClient.call(request)
    .then((JsObject results) {
      model.statements.clear();
      for (JsObject jsStatement in results['rules']) {
        model.statements.add(new Statement.fromJs(jsStatement));
      }
    })
    .catchError((Event error) {
      print('Failed to call /get_all_rules service: $error');
    });
  }
  
  void toast(String text) {
    Element t = this.shadowRoot.querySelector('#toast');
    t.attributes['text'] = text;
    t.show();
  }

  // Creates a blank rule to be filled in.
  void createBlankRule(MouseEvent event) {
    Statement statement = new Statement('unknown', [], '', {}, true);
    model.statements.add(statement);
  }
  
  // Save a new rule to the back end.
  void addRule(Event event, var detail, Element sender) {
    var rule = detail['rule'];
    var statementObject = {
      'id': rule.id,
      'triggers': rule.triggers.map((trigger) => trigger.toJson()),
      'actions': [{'name': rule.action_name, 'params': JSON.encode(rule.action_params)}]
    };
    var request = new ServiceRequest({'rule': statementObject});
    Future future = this._addStatementClient.call(request);
    future.then((JsObject results) {
      detail['rule'].id = results['id'];
      toast('Added new rule.');
    });
  }
  
  // Update an existing rule.
  void updateRule(Event, var detail, Element sender) {
    var rule = detail['rule'];
    var statementObject = {
      'id': rule.id,
      'triggers': rule.triggers.map((trigger) => trigger.toJson()),
      'actions': [{'name': rule.action_name, 'params': JSON.encode(rule.action_params)}]
    };
    var request = new ServiceRequest(
      {
        'id': rule.id,
        'rule': statementObject
      }
    );
    Future future = this._updateStatementClient.call(request);
    future.then((JsObject results) {
      toast('Updated rule.');
    });
  }
  
  void deleteRule(Event event, var detail, Element sender) {
    var request = new ServiceRequest({'id': detail['id']});
    Future future = this._deleteStatementClient.call(request);
    future.then((JsObject results) {
      toast('Deleted rule.');
      
      Timer timer = new Timer(new Duration(milliseconds: 333), () {
        model.statements.removeWhere((Statement card) {
          return card.id == detail['id'];
        });
      });
    });
  }
}
