from dataclasses import dataclass

# Remaps ROS names for a node.
@dataclass
class Remap:
    # Name matching expression to look for replacement candidates.
    from_name: str
    
    # Name replacement expression to replace candidates found.
    to_name: str