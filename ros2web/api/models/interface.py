from dataclasses import dataclass, field
from typing import Optional, ClassVar
from typing import List, Any



@dataclass
class Field:
    name: str
    type: str

    
@dataclass
class Msg:
    type: ClassVar[str] = "msg"
    name: str
    package_name: str
    fields: Optional[List[Field]] = field(default=None)

    @property
    def identifier(self):
        return f'{self.package_name}/{self.type}/{self.name}'
    
@dataclass
class Srv:
    type: ClassVar[str] = "srv"
    name: str
    package_name: str
    request: Optional[List[Field]] = field(default=None)
    response: Optional[List[Field]] = field(default=None)

    @property
    def identifier(self):
        return f'{self.package_name}/{self.type}/{self.name}'

@dataclass
class Act:
    type: ClassVar[str] = "action"
    name: str
    package_name: str
    goal: Optional[List[Field]] = field(default=None)
    result: Optional[List[Field]] = field(default=None)
    feedback: Optional[List[Field]] = field(default=None)
    
    @property
    def identifier(self):
        return f'{self.package_name}/{self.type}/{self.name}'