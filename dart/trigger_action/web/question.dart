import 'package:polymer/polymer.dart';
import 'dart:convert';
import 'dart:core';
import 'dart:html';

@CustomTag('webstudy-question')
class WebstudyQuestionElement extends PolymerElement {
  String participantId;
  String questionId;
  @observable List<String> questionParagraphs;
  
  WebstudyQuestionElement.created() : super.created() {
    Location location = document.window.location;
    Uri uri = Uri.parse(location.href);
    this.participantId = uri.queryParameters['p'];
    this.questionId = uri.queryParameters['q'];
    HttpRequest.getString('/question/$participantId/$questionId').then(onQuestionLoaded);
  }
  
  void onQuestionLoaded(String response) {
    var question = JSON.decode(response);
    questionParagraphs = question['question'].split('\n');
  }
  
  void onNext() {
    
  }
}