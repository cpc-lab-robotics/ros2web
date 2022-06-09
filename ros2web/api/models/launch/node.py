from typing import List, Optional
from dataclasses import dataclass, field

from .env import Env
from .param import Param
from .remap import Remap


@dataclass
class Arg:
    name: str
    value:  Optional[str] = field(default=None)
    default:  Optional[str] = field(default=None)
    description:  Optional[str] = field(default=None)


# Executes a ROS node executable in a local OS process.
@dataclass
class Node:
    pkg: str # Name of the package where the node is to be found.
    exec: str # Name of the node executable.
    name: Optional[str] = field(default=None) # A name for the launched ROS node.
    ros_args: Optional[str] = field(default=None) # ROS-specific 'command-line' arguments for the ROS node.
    args: Optional[str] = field(default=None) # Additional 'command-line' arguments for the ROS node.
    namespace: Optional[str] = field(default=None) # A ROS namespace to scope the launched ROS node.
    launch_prefix: Optional[str] = field(default=None) # A prefix for the ROS node command-line used for launch.
    output: Optional[str] = field(default=None)  # Output type for the launched ROS node. (log or screen)
    # log, ROS node output goes to a log file.
    # screen, ROS node output goes to stdout.


    # A collection of ROS parameters, ROS remappings and/or
    # environment variable settings for the launched ROS node.
    env: Optional[List[Env]] = field(default=None)
    param: Optional[List[Param]] = field(default=None)
    remap: Optional[List[Remap]] = field(default=None)


    # A predicate to condition ROS node launch i.e. only do
    # so if the predicate evaluates to a truthy value.
    if_condition: Optional[str] = field(default=None) 
    
    # A predicate to condition ROS node launch i.e. do so unless
    # the predicate evaluates to a truthy value.
    unless_condition: Optional[str] = field(default=None)
    
    id: Optional[str] = field(default=None)