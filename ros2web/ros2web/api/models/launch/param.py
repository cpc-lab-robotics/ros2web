from typing import List, Optional, Union
from dataclasses import dataclass, field


# Sets a ROS parameter for the enclosing node to a scalar
# value, a sequence of scalar values delimited by a known
# separator or a sequence of nested named parameters, either
# defined in place or in a file to be loaded.
@dataclass
class Param:
    
    # A collection of nested ROS parameters to be set.
    param: Optional['Param'] = field(default=None)
    
    # Name of the ROS parameter to be set.
    name: Optional[str] = field(default=None)

    # Value or list of values to set the the ROS parameter with.
    value: Optional[Union[List[Union[str, int, float]], str, int, float]] = field(default=None)

    # Value separator when dealing with list of values.
    # sep: Optional[str] = field(default=None)

    # Path to the parameters file to be loaded.
    from_path: Optional[str] = field(default=None) 
