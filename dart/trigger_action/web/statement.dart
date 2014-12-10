import 'dart:async';
import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('statement-card')
class StatementElement extends PolymerElement {
  @published Statement model;
  @observable List<Trigger> triggers = toObservable([]);
//  @published String id = 'unknown';
////  @published @observable List<TriggerModel> triggers = toObservable([]);
//  @published String action_name = '';
//  @published Map<String, String> action_params = {};
//  @published bool isNew = false;
  @observable String saveLabel = '';
  @observable String deleteLabel = '';
//  @published List<StatementModel> parentList;
  @observable bool opened = false;

//  @published Ros ros;
//  Service _addStatementClient;
//  Service _updateStatementClient;
//  Service _deleteStatementClient;

  StatementElement.created() : super.created() {
  }

  void attached() {
//    this._addStatementClient = new Service(this.ros, '/add_rule', 'trigger_action_programming/AddRule');
//    this._updateStatementClient = new Service(this.ros, '/update_rule', 'trigger_action_programming/UpdateRule');
//    this._deleteStatementClient = new Service(this.ros, '/delete_rule', 'trigger_action_programming/DeleteRule');
    updateButtons();
    triggers = toObservable(model.triggers);
    Timer timer = new Timer(new Duration(milliseconds: 50), () {
      opened = true;
    });
    if (triggers.length == 0) {
      Trigger trigger = new Trigger('', {}, true);
      triggers.add(trigger);
    }
  }

  

  void updateButtons() {
    if (model.isNew) {
      saveLabel = 'Add rule';
      deleteLabel = 'Cancel';
    } else {
      saveLabel = 'Update rule';
      deleteLabel = 'Delete';
    }
  }
  
  void createTrigger(MouseEvent event) {
    Trigger trigger = new Trigger('', {}, false);
    triggers.add(trigger);
  }

  void save(Event event, Object detail, Element sender) {
    event.preventDefault();
    if (model.isNew) {
      model.triggers = triggers;
      dispatchEvent(
        new CustomEvent(
          'add-rule',
          detail: {'rule': model}
        )
      );
      model.isNew = false;
      updateButtons();
    } else {
      model.triggers = triggers;
      dispatchEvent(
        new CustomEvent(
          'update-rule',
          detail: {'rule': model}
        )
      );
    }
  }

  void delete(Event event, Object detail, Element sender) {
    event.preventDefault();
    if (model.isNew) {
      opened=false;
      Timer timer = new Timer(new Duration(milliseconds: 333), () {
        this.remove();
      });
    } else {
      dispatchEvent(
        new CustomEvent(
          'delete-rule',
          detail: {'id': model.id}
        )
      );
      opened = false;
    }
  }
}

