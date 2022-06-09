from dataclasses import dataclass, field
from typing import Optional
from typing import List, Any

from .interface import Srv


@dataclass
class Service:
    name: str
    type: Srv
