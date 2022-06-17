from typing import Union, Optional, List, Any, Dict, Tuple


import os
import glob
from pathlib import Path
from tinydb import TinyDB, Query, where


class DBBaseException(Exception):
    pass


class DBBase:
    def __init__(self, directory_path):

        if os.path.isdir(directory_path) is False:
            os.mkdir(directory_path)

        self.__path = directory_path
        self.__db_dict = {}

        files = glob.glob(os.path.join(self.__path, "*.json"))
        for file in files:
            db_name = Path(file).stem
            db = TinyDB(os.path.join(self.__path, f"{db_name}.json"))
            self.__db_dict[db_name] = db

    def db(self, db_name) -> TinyDB:
        if db_name not in self.__db_dict:
            db = TinyDB(os.path.join(self.__path, f"{db_name}.json"))
            self.__db_dict[db_name] = db

        return self.__db_dict[db_name]

    def dbs(self):
        return set(self.__db_dict.keys())

    def drop_dbs(self):
        for db_name in self.__db_dict.keys():
            self.drop_db(db_name)

    def drop_db(self, db_name):
        del self.__db_dict[db_name]
        filepath = os.path.join(self.__path, f"{db_name}.json")
        if os.path.isfile(filepath):
            os.remove(filepath)

    def new(self, db, data, criteria) -> bool:
        results = self.find(db, criteria)
        if len(results) == 0:
            db.insert(data)
            return True
        else:
            return False

    def upsert(self, db, data, criteria):
        results = self.find(db, criteria)
        if len(results) > 0:
            db.update(data, Query().fragment(criteria))
        else:
            db.insert(data)

    def insert(self, db, data):
        db.insert(data)

    def insert_multiple(self, db, data: List[Any]):
        db.insert_multiple(data)

    def update(self, db, data, criteria):
        db.update(data, Query().fragment(criteria))

    def remove(self, db, criteria):
        db.remove(Query().fragment(criteria))

    def exist(self, db, field_name):
        return db.get(Query()[field_name].exists())

    def exists(self, db, field_name):
        return db.search(Query()[field_name].exists())

    def get(self, db, criteria):
        return db.get(Query().fragment(criteria))

    def find(self, db, criteria):
        return db.search(Query().fragment(criteria))
