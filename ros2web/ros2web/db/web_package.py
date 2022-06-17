from typing import List, Any
from tinydb import TinyDB, Query


class DBException(Exception):
    pass

class DB:
    def __init__(self, db: TinyDB) -> None:
        self.__db = db

    def new(self, data, criteria) -> bool:
        results = self.find(criteria)
        if len(results) == 0:
            self.insert(data)
            return True
        else:
            return False

    def upsert(self, data, criteria):
        results = self.find(criteria)
        if len(results) > 0:
            self.update(data, Query().fragment(criteria))
        else:
            self.insert(data)

    def insert(self, data):
        self.__db.insert(data)

    def insert_multiple(self, data: List[Any]):
        self.__db.insert_multiple(data)

    def update(self, data, criteria):
        self.__db.update(data, Query().fragment(criteria))

    def remove(self, criteria):
        self.__db.remove(Query().fragment(criteria))

    def exist(self, field_name):
        return self.__db.get(Query()[field_name].exists())

    def exists(self, field_name):
        return self.__db.search(Query()[field_name].exists())

    def get(self, criteria):
        return self.__db.get(Query().fragment(criteria))

    def find(self, criteria):
        return self.__db.search(Query().fragment(criteria))

    @property
    def tiny_db(self) -> TinyDB:
        return self.__db
