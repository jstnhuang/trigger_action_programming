#!/usr/bin/env python
import database

def build(is_mock=False):
    if is_mock:
        db = database.InMemoryProgramDatabase()
        return db
    else:
        db = database.ProgramDatabase(database.DATABASE_LOCATION)
        return db
