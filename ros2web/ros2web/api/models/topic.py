from dataclasses import dataclass, field
from typing import Optional
from typing import List, Any

from .interface import Msg
from .data import Data


@dataclass
class Topic(Data):
    name: str
    type: str
    