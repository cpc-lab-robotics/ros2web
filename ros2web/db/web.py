from pathlib import Path

from . import DBBase

class ROS2WebDBException(Exception):
    pass

class ROS2WebDB(DBBase):
    def __init__(self, directory: str):
        super().__init__(Path(directory).joinpath('db'))