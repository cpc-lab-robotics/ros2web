from dataclasses import dataclass, field
from typing import Optional
from typing import List, Any

import time

from .interface import Srv
from .data import Data


@dataclass
class Service(Data):
    name: str
    type: str
    request_value: Optional[Any] = field(default=None)
    response_value: Optional[Any] = field(default=None)
    descriptor: Optional[Srv] = field(default=None)
    
    def __post_init__(self):
        self.id = f"{self.name}:{self.type}"