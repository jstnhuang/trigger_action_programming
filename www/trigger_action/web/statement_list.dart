import "package:polymer/polymer.dart";
import "roslibjs.dart";
import "statement.dart";
import "dart:async";
import "dart:js";
import "dart:html";

@CustomTag('statement-list')
class StatementList extends PolymerElement {
  var _ros;
  var _getStatementsClient;
  List<Statement> statements = toObservable([]);

  StatementList.created() : super.created() {
  }

  void attached() {
    this._ros = new Ros(robotWebsocketUrl);
    this._getStatementsClient = new Service(this._ros, "/get_all_statements", "trigger_action_programming/GetAllStatements");
    var request = new ServiceRequest({});
    Future future = this._getStatementsClient.call(request);
    future.then((JsObject results) {
      this.statements.clear();
      for (JsObject jsStatement in results['statements']) {
        this.statements.add(new Statement.fromJs(jsStatement));
      }
    });
  }
  
  void createRule(MouseEvent event) {
    var list = shadowRoot.querySelector('#rulelist');
    Element blankStatement = new Element.tag('statement-card');
    blankStatement.attributes['isNew'] = 'true';
    list.append(blankStatement);
  }
}
