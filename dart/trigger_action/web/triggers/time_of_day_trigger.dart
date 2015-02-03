import 'package:polymer/polymer.dart';
import 'package:trigger_action/trigger_model.dart';
import 'dart:html';

@CustomTag('time-of-day-trigger-options')
class TimeOfDayTriggerOptionsElement extends PolymerElement {
  @published TimeOfDayTrigger model;
  @observable bool sunday = false;
  @observable bool monday = false;
  @observable bool tuesday = false;
  @observable bool wednesday = false;
  @observable bool thursday = false;
  @observable bool friday = false;
  @observable bool saturday = false;
  @published bool readOnly = false;
  
  TimeOfDayTriggerOptionsElement.created() : super.created() {
  }
  
  void attached() {
    if (model.days.isEmpty) {
      model.days.addAll(['monday', 'tuesday', 'wednesday', 'thursday', 'friday']);
    }
    sunday = model.days.contains('sunday');
    monday = model.days.contains('monday');
    tuesday = model.days.contains('tuesday');
    wednesday = model.days.contains('wednesday');
    thursday = model.days.contains('thursday');
    friday = model.days.contains('friday');
    saturday = model.days.contains('saturday');
  }
  
  void toggleDay(Event event, Object detail, Element sender) {
    var day = sender.attributes['id'];
    if (day == 'sunday') {
      sunday = !sunday;
      model.days.remove('sunday');
      if (sunday) {
        model.days.add('sunday');
      }
    } else if (day == 'monday') {
      monday = !monday;
      model.days.remove('monday');
      if (monday) {
        model.days.add('monday');
      }
    } else if (day == 'tuesday') {
      tuesday = !tuesday;
      model.days.remove('tuesday');
      if (tuesday) {
        model.days.add('tuesday');
      }
    } else if (day == 'wednesday') {
      wednesday = !wednesday;
      model.days.remove('wednesday');
      if (wednesday) {
        model.days.add('wednesday');
      }
    } else if (day == 'thursday') {
      thursday = !thursday;
      model.days.remove('thursday');
      if (thursday) {
        model.days.add('thursday');
      }
    } else if (day == 'friday') {
      friday = !friday;
      model.days.remove('friday');
      if (friday) {
        model.days.add('friday');
      }
    } else {
      saturday = !saturday;
      model.days.remove('saturday');
      if (saturday) {
        model.days.add('saturday');
      }
    }
  }
}
