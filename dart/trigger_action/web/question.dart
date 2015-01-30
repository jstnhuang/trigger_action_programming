import 'package:polymer/polymer.dart';
import 'dart:convert';
import 'dart:core';
import 'dart:html';

@CustomTag('webstudy-question')
class WebstudyQuestionElement extends PolymerElement {
  @published String api=''; // Default API host is same origin.
  String participantId;
  String questionId;
  @observable List<String> questionParagraphs;
  @observable String questionType;
  HttpRequest request;
  List<String> multipleChoiceOptions;
  @observable String selectedOption = "";
  
  WebstudyQuestionElement.created() : super.created() {
    Location location = document.window.location;
    Uri uri = Uri.parse(location.href);
    this.participantId = uri.queryParameters['p'];
    this.questionId = uri.queryParameters['q'];
    HttpRequest.getString('$api/question/$participantId/$questionId').then(onQuestionLoaded);
  }
  
  void onQuestionLoaded(String questionJson) {
    var question = JSON.decode(questionJson);
    questionType = question['type'];
    questionParagraphs = question['text'].split('\n');
    
    if (questionType == 'understanding') {
      multipleChoiceOptions = question['options'];
      var jsonRules = question['rules'];
      var app = querySelector('trigger-action-app');
      var list = app.shadowRoot.querySelector('statement-list');
      list.model.ruleDb.jsonRules = jsonRules;
      list.reloadRules();
      list.model.readOnly = true;
    }
  }
  
  void saveRules(Event e) {
    e.preventDefault();
    var app = querySelector('trigger-action-app');
    var list = app.shadowRoot.querySelector('statement-list');
    request = new HttpRequest();
    request.onReadyStateChange.listen(onData);

    var url = '$api/next';
    request.open('POST', url);
    request.send(JSON.encode({
      'p': participantId,
      'q': questionId,
      'rules': list.jsonRules()
    }));
  }
  
  void saveMultipleChoice(Event e) {
    e.preventDefault();
    var app = querySelector('trigger-action-app');
    request = new HttpRequest();
    request.onReadyStateChange.listen(onData);

    var url = '$api/next';
    request.open('POST', url);
    request.send(JSON.encode({
      'p': participantId,
      'q': questionId,
      'selected': selectedOption
    }));
  }
  
  void onData(_) {
    if (request.readyState == HttpRequest.DONE &&
        request.status == 200) {
      window.location.href = request.responseText;
    } else if (request.readyState == HttpRequest.DONE &&
        request.status == 0) {
      // TODO: create an error page and redirect to it.
      print('No server');
    }
  }
}