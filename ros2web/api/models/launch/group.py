from typing import List, Optional
from dataclasses import dataclass, field

from .executable import Executable
from .node import Node
from .include import Include
from .let import Let
from .set_env import SetEnv
from .unset_env import UnsetEnv


@dataclass
class Group:
    # The actions that make up the group.
    executable: Optional[List[Executable]] = field(default=None)
    node: Optional[List[Node]] = field(default=None)
    
    include:  Optional[List[Include]] = field(default=None)
    let:  Optional[List[Let]] = field(default=None)
    set_env:  Optional[List[SetEnv]] = field(default=None)
    unset_env:  Optional[List[UnsetEnv]] = field(default=None)

    scoped: Optional[bool] = field(default=None) # Whether the group is a scoping one launch configuration-wise or not.

    # A predicate to condition group launch i.e. only do
    # so if the predicate evaluates to a truthy value.
    if_condition: Optional[str] = field(default=None) 

    # A predicate to condition group launch i.e. do
    # so unless the predicate evaluates to a truthy value.
    unless_condition: Optional[str] = field(default=None) 

    # Groups and optionally scopes a set of actions.
    group: Optional[List['Group']] = field(default=None)
    

    
