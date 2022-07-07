from pathlib import Path
import os

from . import DBBase

class ROS2WebDBException(Exception):
    pass

class ROS2WebDB(DBBase):
    def __init__(self, directory: str):
        directory_path = Path(directory).joinpath('db')
        if os.path.isdir(directory_path) is False:
            os.mkdir(directory_path)
            
        super().__init__(directory_path)