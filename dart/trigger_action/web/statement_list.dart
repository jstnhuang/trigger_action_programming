import 'dart:async';
import 'dart:html';
import 'dart:js';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('statement-list')
class StatementListElement extends PolymerElement {
  @observable StatementList model;
  @published String websocketUrl = "";
  @published bool webstudy = false;
  @published bool readOnly = false;

  StatementListElement.created() : super.created() {
  }
  
  void websocketUrlChanged() {
    if (checkInit()) {
      init();
    }
  }
  
  void webstudyChanged() {
    if (checkInit()) {
      init();
    }
  }
  
  void readOnlyChanged() {
    if (checkInit()) {
      init();
    }
  }
  
  // Initialize only once the websocket URL has been changed at least once.
  bool checkInit() {
    return websocketUrl != "";
  }
  
  void init() {
    model = new StatementList(websocketUrl, webstudy, readOnly);
    model.ruleDb.connect().then((Event event) {
      model.ruleDb.getAllRules().then((List<Statement> results) {
        if (results != null) {
          model.statements = toObservable(results);
        }
      })
      .catchError((var error) {
        print('Failed to call /get_all_rules service: $error');
      });
    })
    .catchError((var error) {
      print('Connection error (possibly temporary): $error');
    });
  }

  void attached() {
    
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
    Statement rule = detail['rule'];
    Future future = model.ruleDb.addRule(rule);
    future.then((String id) {
      detail['rule'].id = id;
      toast('Added new rule.');
    });
  }

  // Update an existing rule.
  void updateRule(Event, var detail, Element sender) {
    Statement rule = detail['rule'];
    Future future = model.ruleDb.updateRule(rule.id, rule);
    future.then((JsObject results) {
      toast('Updated rule.');
    });
  }

  void deleteRule(Event event, var detail, Element sender) {
    Future future = model.ruleDb.deleteRule(detail['id']);
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
