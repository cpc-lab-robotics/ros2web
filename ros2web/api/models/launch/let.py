from dataclasses import dataclass

# Defines a launch configuration variable.

@dataclass
class Let:
    var: str # Name of the launch configuration variable.
    value: str # Value for the launch configuration variable.

