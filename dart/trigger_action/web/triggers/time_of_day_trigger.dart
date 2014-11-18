import 'package:polymer/polymer.dart';
import 'dart:html';

@CustomTag('time-of-day-trigger-options')
class TimeOfDayTriggerOptions extends PolymerElement {
  @published int hour = 12;
  @published int minute = 30;
  @published List days = [];
  @published String timeString = "";
  @published bool sunday = false;
  @published bool monday = false;
  @published bool tuesday = false;
  @published bool wednesday = false;
  @published bool thursday = false;
  @published bool friday = false;
  @published bool saturday = false;
  
  TimeOfDayTriggerOptions.created() : super.created() {
  }
  
  void attached() {
    print(days);
    sunday = days.contains('sunday');
    monday = days.contains('monday');
    tuesday = days.contains('tuesday');
    wednesday = days.contains('wednesday');
    thursday = days.contains('thursday');
    friday = days.contains('friday');
    saturday = days.contains('saturday');
    if (minute < 10) {
      timeString = '$hour:0$minute';
    } else {
      timeString = '$hour:$minute';
    }
  }
  
  void handleTimeChange(Event event, Object detail, Element sender) {
    List<String> parts = timeString.split(':');
    hour = int.parse(parts[0]);
    minute = int.parse(parts[1]);
  }
  
  void toggleDay(Event event, Object detail, Element sender) {
    var day = sender.attributes['id'];
    if (day == 'sunday') {
      sunday = !sunday;
      days.remove('sunday');
      if (sunday) {
        days.add('sunday');
      }
    } else if (day == 'monday') {
      monday = !monday;
      days.remove('monday');
      if (monday) {
        days.add('monday');
      }
    } else if (day == 'tuesday') {
      tuesday = !tuesday;
      days.remove('tuesday');
      if (tuesday) {
        days.add('tuesday');
      }
    } else if (day == 'wednesday') {
      wednesday = !wednesday;
      days.remove('wednesday');
      if (wednesday) {
        days.add('wednesday');
      }
    } else if (day == 'thursday') {
      thursday = !thursday;
      days.remove('thursday');
      if (thursday) {
        days.add('thursday');
      }
    } else if (day == 'friday') {
      friday = !friday;
      days.remove('friday');
      if (friday) {
        days.add('friday');
      }
    } else {
      saturday = !saturday;
      days.remove('saturday');
      if (saturday) {
        days.add('saturday');
      }
    }
  }
}
