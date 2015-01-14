import 'dart:async';
import 'dart:html';
import 'model.dart';
import 'package:polymer/polymer.dart';

@CustomTag('statement-card')
class StatementElement extends PolymerElement {
  @published Statement model;
  @published bool webstudy = false;
  @published bool readOnly = false;
  @observable List<Trigger> triggers = toObservable([]);
  @observable String saveLabel = '';
  @observable String deleteLabel = '';
  @observable bool opened = false;

  StatementElement.created() : super.created() {
  }

  void attached() {
    updateButtons();
    triggers = toObservable(model.triggers);
    Timer timer = new Timer(new Duration(milliseconds: 50), () {
      opened = true;
    });
    if (triggers.length == 0) {
      Trigger trigger = new Trigger('', {});
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
    Trigger trigger = new Trigger('', {});
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
  
  void deleteTrigger(Event, var detail, Element sender) {
    detail['element'].shadowRoot.querySelector('#animation').style.overflow = 'hidden';
    detail['element'].animationStart = false;
    if (detail['element'].isFirst) {
      triggers.removeAt(int.parse(sender.attributes['data-index']));
    } else {
      new Timer(new Duration(milliseconds: 300), () {
        triggers.removeAt(int.parse(sender.attributes['data-index']));
      });
    }
  }
}

