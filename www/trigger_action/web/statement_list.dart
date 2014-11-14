import "package:polymer/polymer.dart";
import "roslibjs.dart";
import "statement.dart";
import "dart:async";
import "dart:js";

@CustomTag('statement-list')
class StatementList extends PolymerElement {
  var _ros;
  var _getStatementsClient;
  List<Statement> statements = toObservable([]);

  StatementList.created() : super.created() {
  }

  void attached() {
    this._ros = new Ros("ws://c1.cs.washington.edu:9999");
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
}
