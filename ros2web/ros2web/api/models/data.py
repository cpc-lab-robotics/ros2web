from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Data:
    id: str = field(init=False)
    