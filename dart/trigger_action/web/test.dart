import 'package:unittest/unittest.dart';
import 'package:trigger_action/model.dart';
import 'package:trigger_action/rule_db.dart';

void main() {
  group('toJSON', () {
    test('StatementList.toJSON common case', () {
      Trigger t1 = new Trigger('person_detected', {});
      Trigger t2 = new Trigger('person_detected2', {});
      Statement s1 = new Statement('test id', [t1, t2], 'say_something', {'speech': 'hello world'}, false);
      Trigger t3 = new Trigger('person_detected', {});
      Statement s2 = new Statement('test id', [t3], 'say_something', {'speech': 'hello world'}, false);
      StatementList list = new StatementList.fromDependencies(new NullRuleDb(), false, false);
      list.statements = [s1, s2];
      Object expected = {
        'rules': [s1, s2]
      };
      expect(list.toJson(), equals(expected));
    });
    test('Statement.toJSON common case', () {
      Trigger t1 = new Trigger('person_detected', {});
      Trigger t2 = new Trigger('person_detected2', {});
      Statement s = new Statement('test id', [t1, t2], 'say_something', {'speech': 'hello world'}, false);
      Object expected = {
        'id': 'test id',
        'triggers': [t1, t2],
        'actions': [
          {
            'name': 'say_something',
            'params': {'speech': 'hello world'}
          }
        ]
      };
      expect(s.toJson(), equals(expected));
    });
    test('Trigger.toJSON common case', () {
      Trigger t = new Trigger('person_detected', {});
      Object expected = {
        'name': 'person_detected',
        'params': {}
      };
      expect(t.toJson(), equals(expected));
    });
    test('Action.toJSON common case', () {
      Action a = new Action('say_something', {'speech': 'hello world'});
      Object expected = {
        'name': 'say_something',
        'params': {
          'speech': 'hello world'
        }
      };
      expect(a.toJson(), equals(expected));
    });
  });
}