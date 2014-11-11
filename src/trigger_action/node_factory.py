#!/usr/bin/env python
import database_factory
import node

def build(name, is_mock=False):
    db = database_factory.build(is_mock)
    return node.Node(name, db)
