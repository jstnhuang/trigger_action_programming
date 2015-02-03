library trigger_action_model;

import 'dart:convert';
import 'dart:js';
import 'package:polymer/polymer.dart';
import 'package:trigger_action/action_model.dart';
import 'package:trigger_action/trigger_model.dart';
import 'roslibjs.dart';
import 'rule_db.dart';

// A result to return from validate.
class ValidationResult {
  bool isValid = false;
  String message = '';
  
  ValidationResult(this.isValid, {this.message: ''});
}

class TriggerActionApp extends Observable {
  RuleDb ruleDb;
  @observable bool isConnected;
  String websocketUrl;
  bool webstudy;
  bool readOnly;
  TriggerActionApp.fromDependencies(this.websocketUrl, this.ruleDb, this.webstudy, this.readOnly);
  factory TriggerActionApp(String websocketUrl, bool webstudy, bool readOnly) {
    if (webstudy) {
      RuleDb ruleDb = new MockRuleDb();
      return new TriggerActionApp.fromDependencies('none', ruleDb, webstudy, readOnly);
    } else {
      Ros ros = new Ros(websocketUrl);
      RuleDb ruleDb = new RosRuleDb(ros);
      return new TriggerActionApp.fromDependencies(websocketUrl, ruleDb, webstudy, readOnly);
    }
  }
}

class StatementList extends Observable {
  @observable List<Statement> statements = toObservable([]);
  RuleDb ruleDb;
  bool webstudy;
  @observable bool readOnly;
  StatementList.fromDependencies(this.ruleDb, this.webstudy, this.readOnly);
  factory StatementList(String websocketUrl, bool webstudy, bool readOnly) {
    if (webstudy) {
      RuleDb ruleDb = new MockRuleDb();
      return new StatementList.fromDependencies(ruleDb, webstudy, readOnly);
    } else {
      Ros ros = new Ros(websocketUrl);
      RuleDb ruleDb = new RosRuleDb(ros);
      return new StatementList.fromDependencies(ruleDb, webstudy, readOnly);
    }
  }
  toJson() {
    List ss = new List.from(this.statements.map((e) => e.toJson()));
    return ss;
  }
  
  // Return whether the statement list is valid.
  ValidationResult validate() {
    // For the webstudy, a response is required.
    if (webstudy && statements.isEmpty) {
      return new ValidationResult(false, message: 'There must be at least one rule.');
    }
    for (Statement statement in statements) {
      ValidationResult vr = statement.validate();
      if (!vr.isValid) {
        return vr;
      }
    }
    return new ValidationResult(true);
  }
}

class Statement extends Observable {
  @observable String id;
  @observable List<Trigger> triggers = toObservable([]);
  @observable Action action;
  @observable bool isNew = false;
  Statement(this.id, this.triggers, this.action, this.isNew);
  Statement.fromJs(JsObject jsStatement)
    : id = jsStatement['id'],
      triggers = toObservable([]) {
    String action_name = jsStatement['actions'][0]['name'];
    var action_params =  JSON.decode(jsStatement['actions'][0]['params']);
    this.action = new Action(action_name, action_params);
    for (var obj in jsStatement['triggers']) {
      String name = obj['name'];
      Map<String, String> params = JSON.decode(obj['params']);
      Trigger trigger = new Trigger(name, params);
      triggers.add(trigger);
    }
  }
//  Statement.fromDecodedJson(var json)
//    : id = json['id'],
//      triggers = [],
//      action_name = json['actions'][0]['name'],
//      action_params = JSON.decode(json['actions'][0]['params']) {
//    action_name = json['actions'][0]['name'];
//    action_params = JSON.decode(json['actions'][0]['params']);
//    this.action = new Action(action_name, action_params);
//    for (var obj in json['triggers']) {
//      String name = obj['name'];
//      Map<String, String> params = JSON.decode(obj['params']);
//      Trigger trigger = new Trigger(name, params);
//      triggers.add(trigger);
//    }
//  }
  toJson() {
    List ts = new List.from(triggers.map((trigger) => trigger.toJson()));
    return {
      'id': this.id,
      'triggers': ts,
      'actions': [action.toJson()]
    };
  }
  
  ValidationResult validate() {
    if (triggers.length == 0) {
      return new ValidationResult(false, message: 'Rule needs at least one trigger.');
    }
    for (Trigger trigger in triggers) {
      ValidationResult vr = trigger.validate();
      if (!vr.isValid) {
        return vr;
      }
    }
    return action.validate();
  }
}

class Trigger extends Observable {
  @observable String name;
  @observable var params;
  @observable var currentTrigger; // Trigger model, which can vary at runtime.
  
  Trigger(this.name, this.params) {
    this.currentTrigger = triggerFactory(this.name, this.params);
  }
  
  Trigger.fromJs(JsObject obj) {
    name = obj['name'];
    params = JSON.decode(obj['params']);
    currentTrigger = triggerFactory(name, params);
  }
  
  toJson() {
    return currentTrigger.toJson();
  }
  
  ValidationResult validate() {
    return currentTrigger.validate();
  }
}

Object triggerFactory(String name, var params) {
  if(name == 'time_of_day') {
    int hour = params['hour'];
    int minute = params['minute'];
    List days = toObservable(params['days']);
    String beforeOrAfter = params['before_or_after'];
    return new TimeOfDayTrigger(hour, minute, days, beforeOrAfter);
  } else if (name == 'person_detected') {
    return new PersonDetectedTrigger();
  }
  return null;
}

class Action extends Observable {
  @observable String name;
  @observable var params;
  @observable var currentAction;
  Action(this.name, this.params) {
    this.currentAction = actionFactory(name, params);
  }
  Action.fromJs(JsObject obj) {
    name = obj['name'];
    params = JSON.decode(obj['params']);
    this.currentAction = actionFactory(name, params);
  }
  toJson() {
    return currentAction.toJson();
  }
  ValidationResult validate() {
    return currentAction.validate();
  }
}

Object actionFactory(String name, Map<String, String> params) {
  if(name == 'say_something') {
    return new SaySomethingAction(params['speech']);
  }
  return null;
}