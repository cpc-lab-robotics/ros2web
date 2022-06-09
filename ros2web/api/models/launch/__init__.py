from typing import List, Optional
from dataclasses import dataclass, field

from .executable import Executable
from .node import Node
from .include import Include
from .group import Group
from .let import Let
from .set_env import SetEnv
from .unset_env import UnsetEnv


@dataclass
class Arg:
    name: str # Name of the launch argument.
    value: Optional[str] = field(default=None) # Fixed value for the launch argument, disables overrides.
    default: Optional[str] = field(default=None) # Default value for the launch argument, used if non is provided.
    description : Optional[str] = field(default=None) # Brief description of the launch argument.


@dataclass
class Launch:
    version: str = field(default="0.1.0") # Launch XML schema version in use. value="[0-9]+\.[0-9]+\.[0-9]+"

    arg: Optional[List[Arg]] = field(default=None)
    let: Optional[List[Let]] = field(default=None)
    node: Optional[List[Node]] = field(default=None)
    executable: Optional[List[Executable]] = field(default=None)
    include: Optional[List[Include]] = field(default=None)
    group: Optional[List[Group]] = field(default=None)
    set_env: Optional[List[SetEnv]] = field(default=None)
    unset_env: Optional[List[UnsetEnv]] = field(default=None)

   