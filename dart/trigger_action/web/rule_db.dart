import 'dart:async';
import 'dart:convert';
import 'dart:js';
import 'model.dart';
import 'roslibjs.dart';

// The CRUD interface for rules, which can be implemented by a ROS service,
// App Engine datastore, etc.
abstract class RuleDb {
  Future connect();
  // Returns a Future with a List<Statement> of results.
  Future getAllRules();
  // Returns a Future with the ID of the added rule.
  Future addRule(Statement rule);
  // Returns a Future with no result.
  Future updateRule(String id, Statement rule);
  // Returns a Future with no result.
  Future deleteRule(String id);
}

class RosRuleDb implements RuleDb {
  Ros _ros;
  Service _getRulesClient;
  Service _addRuleClient;
  Service _updateRuleClient;
  Service _deleteRuleClient;

  RosRuleDb(this._ros) {
    _getRulesClient = new Service(this._ros, '/get_all_rules', 'trigger_action_programming/GetAllRules');
    _addRuleClient = new Service(this._ros, '/add_rule', 'trigger_action_programming/AddRule');
    _updateRuleClient = new Service(this._ros, '/update_rule', 'trigger_action_programming/UpdateRule');
    _deleteRuleClient = new Service(this._ros, '/delete_rule', 'trigger_action_programming/DeleteRule');
  }

  Future connect() {
    return _ros.connect().then((var event) {
      _getRulesClient.connect();
      _addRuleClient.connect();
      _updateRuleClient.connect();
      _deleteRuleClient.connect();
    });
  }

  Future getAllRules() {

    var request = new ServiceRequest({});
    return _getRulesClient.call(request).then(
      (JsObject results) {
        List<Statement> rules = [];
        for (JsObject jsStatement in results['rules']) {
          rules.add(new Statement.fromJs(jsStatement));
        }
        return rules;
      }
    );
  }

  Future addRule(Statement rule) {
    var statementObject = {
      'id': rule.id,
      'triggers': rule.triggers.map((trigger) => trigger.toJson()),
      'actions': [{'name': rule.action_name, 'params': JSON.encode(rule.action_params)}]
    };
    var request = new ServiceRequest({'rule': statementObject});
    Future future = this._addRuleClient.call(request);
    return future.then(
      (JsObject results) {
        return results['id'];
      }
    );
  }

  Future updateRule(String id, Statement rule) {
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
    return this._updateRuleClient.call(request);
  }

  Future deleteRule(String id) {
    var request = new ServiceRequest({'id': id});
    return this._deleteRuleClient.call(request);
  }
}