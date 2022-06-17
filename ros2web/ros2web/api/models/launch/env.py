from dataclasses import dataclass


# Modifies an executable process environment.
@dataclass
class Env:
    name: str # Name of the environment variable to be set.
    value: str # Value to be set for the environment variable.