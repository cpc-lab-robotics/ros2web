from typing import Optional
from dataclasses import dataclass, field


# Modifies an executable process environment.
@dataclass
class UnsetEnv:
    name: str # Name of the environment variable to be set.

    # A predicate to condition executable launch i.e. only do
    # so if the predicate evaluates to a truthy value.
    if_condition: Optional[str] = field(default=None) 
    
    # A predicate to condition executable launch i.e. do so unless
    # the predicate evaluates to a truthy value.
    unless_condition: Optional[str] = field(default=None)
