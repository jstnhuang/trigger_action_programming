library trigger_action_model;

import 'dart:convert';
import 'dart:js';
import 'package:polymer/polymer.dart';
import 'roslibjs.dart';
import 'rule_db.dart';

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
    return this.statements;
  }
}

class Statement extends Observable {
  @observable String id;
  @observable List<Trigger> triggers = toObservable([]);
  @observable String action_name;
  @observable Map<String, String> action_params;
  @observable bool isNew = false;
  Statement(this.id, this.triggers, this.action_name, this.action_params, this.isNew);
  Statement.fromJs(JsObject jsStatement)
    : id = jsStatement['id'],
      triggers = [],
      action_name = jsStatement['actions'][0]['name'],
      action_params =  JSON.decode(jsStatement['actions'][0]['params']) {
    for (var obj in jsStatement['triggers']) {
      String name = obj['name'];
      Map<String, String> params = JSON.decode(obj['params']);
      Trigger trigger = new Trigger(name, params);
      triggers.add(trigger);
    }
  }
  Statement.fromDecodedJson(var json)
    : id = json['id'],
      triggers = [],
      action_name = json['actions'][0]['name'],
      action_params = JSON.decode(json['actions'][0]['params']) {
    for (var obj in json['triggers']) {
      String name = obj['name'];
      Map<String, String> params = JSON.decode(obj['params']);
      Trigger trigger = new Trigger(name, params);
      triggers.add(trigger);
    }
  }
  toJson() {
    return {
      'id': this.id,
      'triggers': triggers.map((trigger) => trigger.toJson()),
      'actions': [{'name': this.action_name, 'params': JSON.encode(this.action_params)}]
    };
  }
}

class Trigger extends Observable {
  @observable String name;
  @observable Map<String, String> params;
  Trigger(this.name, this.params);
  Trigger.fromJs(JsObject obj)
    : name = obj['name'],
      params = JSON.decode(obj['params']) {
  }
  toJson() {
    return {
      'name': name,
      'params': JSON.encode(params)
    };
  }
}

class Action extends Observable {
  @observable String name;
  @observable Map<String, String> params;
  Action(this.name, this.params);
  Action.fromJs(JsObject obj)
    : name = obj['name'],
      params = JSON.decode(obj['params']) {
  }
  toJson() {
    return {
      'name': name,
      'params': JSON.encode(params)
    };
  }
}