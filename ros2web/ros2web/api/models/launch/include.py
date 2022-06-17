from typing import List, Optional
from dataclasses import dataclass, field


# An included launch file argument provision.
@dataclass
class Arg:
    name: str # Name of the launch argument.
    value: str # Value for the launch argument.

# Arguments for the included launch file, if any.
@dataclass
class Include:
    file: str # Path to the launch file to be included.

    arg: Optional[List[Arg]] = field(default=None)
    
    # A predicate to condition launch inclusion 
    # i.e. only do so if the predicate evaluates to a truthy value.
    if_condition: Optional[str] = field(default=None) 

    # A predicate to condition launch inclusion i.e. do
    # so unless the predicate evaluates to a truthy value.
    unless_condition: Optional[str] = field(default=None)

    id: Optional[str] = field(default=None)