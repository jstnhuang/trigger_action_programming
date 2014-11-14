library roslibjs;

import "dart:async";
import "dart:js";

class Ros {
  var jsRos;
  
  Ros(var url) {
    var params = new JsObject.jsify({"url": url});
    this.jsRos = new JsObject(context['ROSLIB']['Ros'], [params]);
    var connectionCallback = new JsFunction.withThis(_handleConnection);
    this.jsRos.callMethod("on", ["connection", connectionCallback]);
    var errorCallback = new JsFunction.withThis(_handleError);
    this.jsRos.callMethod("on", ["error", errorCallback]);
    var closeCallback = new JsFunction.withThis(_handleClose);
    this.jsRos.callMethod("on", ["close", closeCallback]);
  }
    
  void _handleConnection(jsThis, event) {
    print("Connected to websocket server.");
  }
  
  void _handleError(jsThis, error) {
    print("Error connecting to websocket server: $error");
  }
  
  void _handleClose(jsThis, event) {
    print("Connection to websocket server closed.");
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