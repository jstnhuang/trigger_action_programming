import 'package:polymer/polymer.dart';

@CustomTag('time-picker')
class TimeOfDayTriggerOptions extends PolymerElement {
  @published int hour; // The true hour, from 0 - 23
  @published int minute;
  @published bool disabled = false;
  
  @observable String hourView = ''; // How the hour is displayed.
  @observable String minuteView = '';
  @observable String amPmView = '';
  @observable bool isValid = true;
  @observable String value = '';
  @observable String errorMsg = '';
  bool isFocused = false;
  
  TimeOfDayTriggerOptions.created() : super.created() {
  }
  
  void hourChanged() {
    if (hour == 0) {
      hourView = '12';
      amPmView = 'AM';
    } else if (hour < 10) {
      hourView = '0$hour';
      amPmView = 'AM';
    } else if (hour < 12) {
      hourView = '$hour';
      amPmView = 'AM';
    } else if (hour == 12) {
      hourView = '$hour';
      amPmView = 'PM';
    } else {
      int hourMod = hour % 12;
      if (hourMod < 10) {
        hourView = '0$hourMod';
      } else {
        hourView = '$hourMod';
      }
      amPmView = 'PM';
    }
    if (!isFocused) {
      value = '$hourView:$minuteView $amPmView';
    }
  }
  
  void minuteChanged() {
    if (minute < 10) {
      minuteView = '0$minute';
    } else {
      minuteView = '$minute';
    }
    if (!isFocused) {
      value = '$hourView:$minuteView $amPmView';
    }
  }
  
  void setFocus() {
    isFocused = true;
  }
  
  void canonicalize() {
    isFocused = false;
    if (validate(value)) {
      value = '$hourView:$minuteView $amPmView';
    }
  }
  
  bool validate(String value) {
    RegExp exp = new RegExp(r"^(\d?\d):(\d\d)\s?([pPaA][mM]?){0,1}$");
    Match match = exp.firstMatch(value);
    if (match == null) {
      errorMsg = 'Time must be in this format: 11:30 PM';
      return false;
    }
    String hourMatch = match.group(1);
    String minuteMatch = match.group(2);
    
    int hourVal = 0;
    int minuteVal = 0;
    try {
      hourVal = int.parse(hourMatch);
      minuteVal = int.parse(minuteMatch);
    } catch (e) {
      errorMsg = 'Hour and/or minute must be numbers';
      return false;
    }
    if (hourVal < 1 || hourVal > 23) {
      errorMsg = 'Hour must be between 1 and 23';
      return false;
    }
    if (minuteVal < 0 || minuteVal > 59) {
      errorMsg = 'Minute must be between 00 and 59';
      return false;
    }
    
    String amPmVal = '';
    if (match.group(3) == null) {
      if (hourVal < 13) {
        errorMsg = 'Must specify AM or PM';
        return false;
      } else {
        amPmVal = 'PM';
      }
    } else {
      amPmVal = match.group(3).toUpperCase();
      if (amPmVal != 'AM' && amPmVal != 'PM' && amPmVal != 'A' && amPmVal != 'P') {
        // The regex should actually prevent this case.
        errorMsg = 'AM / PM was not spelled correctly';
        return false;
      }
      if (!amPmVal.endsWith('M')) {
        amPmVal += 'M';
      }
      if (amPmVal == 'AM' && hourVal > 12) {
        errorMsg = 'The hour is too late to be an AM time';
        return false;
      }
    }
    
    if (hourVal < 12 && amPmVal == 'PM') {
      hour = hourVal + 12;
    } else if (hourVal == 12 && amPmVal == 'AM') {
      hour = 0;
    } else {
      hour = hourVal;
    }
    
    minute = minuteVal;
    amPmView = amPmVal;
    return true;
  }
  
  void valueChanged() {
    isValid = validate(value);
  }
  
  void attached() {
    
  }
}
