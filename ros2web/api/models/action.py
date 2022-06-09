from dataclasses import dataclass, field
from typing import Optional
from typing import List, Any

from .interface import Act


@dataclass
class Action:
    name: str
    type: Act