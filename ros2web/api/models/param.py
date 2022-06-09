from dataclasses import dataclass, field
from typing import Optional
from typing import List, Any
from enum import Enum


class ParamType(Enum):
    NOT_SET = 'PARAMETER_NOT_SET'
    BOOL = 'PARAMETER_BOOL'
    INTEGER = 'PARAMETER_INTEGER'
    DOUBLE = 'PARAMETER_DOUBLE'
    STRING = 'PARAMETER_STRING'
    BYTE_ARRAY = 'PARAMETER_BYTE_ARRAY'
    BOOL_ARRAY = 'PARAMETER_BOOL_ARRAY'
    INTEGER_ARRAY = 'PARAMETER_INTEGER_ARRAY'
    DOUBLE_ARRAY = 'PARAMETER_DOUBLE_ARRAY'
    STRING_ARRAY = 'PARAMETER_STRING_ARRAY'


@dataclass
class FloatingPointRange:
    from_value: float
    to_value: float
    step: float


@dataclass
class IntegerRange:
    from_value: int
    to_value: int
    step: int


@dataclass
class ParamDescriptor:
    name: str
    type: ParamType
    description: str

    additional_constraints: Optional[str] = field(default=None)
    read_only: Optional[bool] = field(default=False)
    dynamic_typing: Optional[bool] = field(default=False)
    floating_point_range: Optional[FloatingPointRange] = field(default=False)
    integer_range: Optional[IntegerRange] = field(default=False)


@dataclass
class Param:
    name: str
    value: Any
    descriptor: Optional[ParamDescriptor] = field(default=None)
