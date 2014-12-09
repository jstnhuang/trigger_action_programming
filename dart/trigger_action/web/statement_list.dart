import "package:polymer/polymer.dart";
import "roslibjs.dart";
import "statement.dart";
import "dart:js";
import "dart:html";

class NullTreeSanitizer implements NodeTreeSanitizer {
  void sanitizeTree(node) {}
}

@CustomTag('statement-list')
class StatementList extends PolymerElement {
  @published Ros ros;
  Service _getStatementsClient;
  List<StatementModel> statements = toObservable([]);

  StatementList.created() : super.created() {
  }

  void attached() {
    _getStatementsClient = new Service(this.ros, "/get_all_statements", "trigger_action_programming/GetAllStatements");

    var request = new ServiceRequest({});
    _getStatementsClient.call(request)
    .then((JsObject results) {
      this.statements.clear();
      for (JsObject jsStatement in results['statements']) {
        this.statements.add(new StatementModel.fromJs(jsStatement));
      }
    })
    .catchError((Event error) {
      print('Failed to call /get_all_statements service: $error');
    });
  }

  void createRule(MouseEvent event) {
    StatementModel statement = new StatementModel('unknown', [], '', {}, true);
    statements.add(statement);
  }
}
