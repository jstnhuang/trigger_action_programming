library trigger_action.web.models;

import 'dart:convert';
import 'dart:js';
import 'package:polymer/polymer.dart';
import 'roslibjs.dart';
import 'rule_db.dart';

class TriggerActionApp extends Observable {
  RuleDb ruleDb;
  @observable bool isConnected;
  String websocketUrl;
  TriggerActionApp.fromDependencies(this.websocketUrl, this.ruleDb);
  factory TriggerActionApp(String websocketUrl) {
    Ros ros = new Ros(websocketUrl);
    RuleDb ruleDb = new RosRuleDb(ros);
    return new TriggerActionApp.fromDependencies(websocketUrl, ruleDb);
  }
}

class StatementList extends Observable {
  @observable List<Statement> statements = toObservable([]);
  RuleDb ruleDb;
  StatementList.fromDependencies(this.ruleDb);
  factory StatementList(String websocketUrl) {
    Ros ros = new Ros(websocketUrl);
    RuleDb ruleDb = new RosRuleDb(ros);
    return new StatementList.fromDependencies(ruleDb);
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
    for (JsObject obj in jsStatement['triggers']) {
      String name = obj['name'];
      Map<String, String> params = JSON.decode(obj['params']);
      Trigger trigger = new Trigger(name, params);
      triggers.add(trigger);
    }
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
  @observable bool isFirst = false;
  Action(this.name, this.params, this.isFirst);
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