library roslibjs;

import "dart:async";
import "dart:js";

String robotWebsocketUrl = 'ws://c1.cs.washington.edu:9999';
String localWebsocketUrl = 'ws://localhost:9999';

class Ros {
  JsObject params;
  JsObject jsRos;
  
  Ros(String url) {
    params = new JsObject.jsify({"url": url});
  }
  
  Future connect() {
    Completer completer = new Completer();
    void _handleConnection(jsThis, event) {
      completer.complete(event);
    }
    void _handleError(jsThis, error) {
      completer.completeError(error);
    }
    void _handleClose(jsThis, event) {
      print("Connection to websocket server closed.");
    }
    this.jsRos = new JsObject(context['ROSLIB']['Ros'], [params]);
    var connectionCallback = new JsFunction.withThis(_handleConnection);
    this.jsRos.callMethod("on", ["connection", connectionCallback]);
    var errorCallback = new JsFunction.withThis(_handleError);
    this.jsRos.callMethod("on", ["error", errorCallback]);
    var closeCallback = new JsFunction.withThis(_handleClose);
    this.jsRos.callMethod("on", ["close", closeCallback]);
    return completer.future;
  }
}

class Service {
  var _ros;
  var _name;
  var _type;
  var _service;
  Service(var ros, var name, var type) {
    this._ros = ros;
    this._name = name;
    this._type = type;
    var params = new JsObject.jsify({
      "ros": ros.jsRos,
      "name": name,
      "type": type
    });
    this._service = new JsObject(context["ROSLIB"]["Service"], [params]);
  }
  Future call(ServiceRequest request) {
    Completer completer = new Completer();
    void handleResult(jsThis, result) {
      completer.complete(result);
    }
    void handleError(jsThis, error) {
      completer.completeError(error);
    }
    var callback = new JsFunction.withThis(handleResult);
    var errorCallback = new JsFunction.withThis(handleError);
    this._service.callMethod("callService", [request.jsServiceRequest, callback, errorCallback]);
    return completer.future;
  }
}

class ServiceRequest {
  var jsServiceRequest;
  ServiceRequest(obj) {
    this.jsServiceRequest = new JsObject.jsify(obj);
  }
}
