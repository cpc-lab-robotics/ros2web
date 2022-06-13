from dataclasses import dataclass, field
from typing import Optional, ClassVar, Union
from typing import List, Any
from .data import Data


@dataclass
class Field:
    type: str
    name: str
    nested: Optional[List['Field']] = field(default=None)


@dataclass
class Msg(Data):
    type: ClassVar[str] = "msg"
    name: str
    package_name: str
    fields: Optional[List[Field]] = field(default=None)

    def __post_init__(self):
        self.id = f'{self.package_name}/{self.type}/{self.name}'


@dataclass
class Srv(Data):
    type: ClassVar[str] = "srv"
    name: str
    package_name: str
    request: Optional[List[Field]] = field(default=None)
    response: Optional[List[Field]] = field(default=None)

    def __post_init__(self):
        self.id = f'{self.package_name}/{self.type}/{self.name}'


@dataclass
class Act(Data):
    type: ClassVar[str] = "action"
    name: str
    package_name: str
    goal: Optional[List[Field]] = field(default=None)
    result: Optional[List[Field]] = field(default=None)
    feedback: Optional[List[Field]] = field(default=None)

    def __post_init__(self):
        self.id = f'{self.package_name}/{self.type}/{self.name}'
