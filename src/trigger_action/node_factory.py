#!/usr/bin/env python
import database_factory
import node

def build(name, is_mock):
    db = database_factory.build(is_mock)
    if is_mock:
        return node.Node(name, db, is_mock)
    else:
        return node.DatabaseOnlyNode(name, db, is_mock)
