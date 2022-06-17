from typing import List, Optional
from dataclasses import dataclass, field

from .env import Env

# Executes an executable as a local OS process.
@dataclass
class Executable:
    # Path to the executable or a command-line if a
    # shell is used to launch.
    cmd: str
    cwd: Optional[str] = field(default=None) # The working directory for the launched process.
    name: Optional[str] = field(default=None) # A name or label for the launched executable.
    args: Optional[str] = field(default=None) # Additional 'command-line' arguments for the executable.
    shell: Optional[bool] = field(default=None) # Whether to use a shell to launch or not.
    launch_prefix: Optional[str] = field(default=None) # A prefix for the executable command-line if a shell is used to launch.
    output: Optional[str] = field(default=None)  # Output type for the launched executable. (log or screen)
    # log, Executable output goes to a log file.
    # screen, Executable output goes to stdout.

    # A collection of environment variable settings for the
    # launched executable.
    env: Optional[List[Env]] = field(default=None) 

    # A predicate to condition executable launch i.e. only do
    # so if the predicate evaluates to a truthy value.
    if_condition: Optional[str] = field(default=None) 
    
    # A predicate to condition executable launch i.e. do so unless
    # the predicate evaluates to a truthy value.
    unless_condition: Optional[str] = field(default=None)