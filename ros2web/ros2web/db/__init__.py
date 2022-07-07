from typing import Set, Union, Optional, List, Any, Dict, Tuple


import os
import glob
from pathlib import Path
from tinydb import TinyDB, Query, where
from tinydb.table import Document, Table

class DBBaseException(Exception):
    pass


class DBBase:
    def __init__(self, path):
        self.__path = path
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

    def dbs(self) -> Set:
        return set(self.__db_dict.keys())

    def drop_dbs(self):
        for db_name in self.__db_dict.keys():
            self.drop_db(db_name)

    def drop_db(self, db_name: str):
        del self.__db_dict[db_name]
        filepath = os.path.join(self.__path, f"{db_name}.json")
        if os.path.isfile(filepath):
            os.remove(filepath)

    def new(self, table: Table, data: Dict, criteria: Dict) -> bool:
        results = self.find(table, criteria)
        if len(results) == 0:
            table.insert(data)
            return True
        else:
            return False

    def upsert(self, table: Table, data:Dict, criteria:Dict):
        results = self.find(table, criteria)
        if len(results) > 0:
            table.update(data, Query().fragment(criteria))
        else:
            table.insert(data)

    def insert(self, table: Table, data: Dict):
        table.insert(data)

    def insert_multiple(self, table: Table, data: List[Dict]):
        table.insert_multiple(data)

    def update(self, table: Table, data: Dict, criteria: Dict):
        table.update(data, Query().fragment(criteria))

    def remove(self, table: Table, criteria: Dict):
        table.remove(Query().fragment(criteria))

    def exist(self, table: Table, field_name: str):
        return table.get(Query()[field_name].exists())

    def exists(self, table: Table, field_name: str):
        return table.search(Query()[field_name].exists())

    def get(self, table: Table, criteria: Dict):
        return table.get(Query().fragment(criteria))

    def find(self, table: Table, criteria: Dict):
        return table.search(Query().fragment(criteria))
    