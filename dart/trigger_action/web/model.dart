library trigger_action.web.models;

import 'dart:convert';
import 'dart:js';
import 'package:polymer/polymer.dart';
import 'roslibjs.dart';
import 'rule_db.dart';

class TriggerActionApp extends Observable {
  RuleDb ruleDb;
  @observable bool isConnected;
  String type;
  TriggerActionApp.fromDependencies(this.type, this.ruleDb);
  factory TriggerActionApp(String type) {
    if (type == 'local_robot') {
      // Construct a trigger action app for a local robot.
      Ros ros = new Ros('ws://walle.cs.washington.edu:9999');
      RuleDb ruleDb = new RosRuleDb(ros);
      return new TriggerActionApp.fromDependencies(type, ruleDb);
    } else if (type == 'real_robot') {
      // Construct a trigger action app for a real robot.
      Ros ros = new Ros(robotWebsocketUrl);
      RuleDb ruleDb = new RosRuleDb(ros);
      return new TriggerActionApp.fromDependencies(type, ruleDb);
    } else {
      throw new ArgumentError.value(type, 'type', 'Unknown TriggerActionApp type');
    }
  }
}

class StatementList extends Observable {
  @observable List<Statement> statements = toObservable([]);
  RuleDb ruleDb;
  StatementList.fromDependencies(this.ruleDb);
  factory StatementList(String type) {
    if (type == 'local_robot') {
      // Construct a trigger action app for a local robot.
      Ros ros = new Ros('ws://walle.cs.washington.edu:9999');
      RuleDb ruleDb = new RosRuleDb(ros);
      return new StatementList.fromDependencies(ruleDb);
    } else if (type == 'real_robot') {
      // Construct a trigger action app for a real robot.
      Ros ros = new Ros(robotWebsocketUrl);
      RuleDb ruleDb = new RosRuleDb(ros);
      return new StatementList.fromDependencies(ruleDb);
    } else {
      throw new ArgumentError.value(type, 'type', 'Unknown StatementList type');
    }
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