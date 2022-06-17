from enum import Enum
import dataclasses
from dataclasses import dataclass
from dacite import from_dict

from ...utilities.re import snake_to_camle, camel_to_snake
from ...utilities.converter import props_to_camel, props_to_snake

from .node import Node
from .package import PackageManifest
from .param import Param
from .interface import Srv, Msg, Act, Field
from .topic import Topic
from .service import Service
from .action import Action

CLASS_MAP = {
    'Node': Node,
    'PackageManifest': PackageManifest,
    'Param': Param,
    'Msg': Msg,
    'Srv': Srv,
    'Act': Act,
    'Topic': Topic,
    'Service': Service,
    'Action': Action,
}

def dict_to_field(data):
    if isinstance(data, dict):
        return [Field(name=name, type=dict_to_field(f)) for name, f in data.items()]
    else:
        if isinstance(data, bytes):
            type = 'bytes'
        if isinstance(data, int):
            type = 'int'
        elif isinstance(data, float):
            type = 'float'
        elif isinstance(data, str):
            type = 'str'
        else:
            type = 'unkown'
        return type
    

def custom_asdict_factory(data):
    def convert_value(obj):
        if isinstance(obj, Enum):
            return obj.value
        return obj
    return dict((k, convert_value(v)) for k, v in data)


def dataclasses_to_dict(data):
    if isinstance(data, dict):
        return {k: dataclasses_to_dict(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [dataclasses_to_dict(v) for v in data]
    else:
        if dataclasses.is_dataclass(data):
            data = dataclasses.asdict(data)
        return data

def model_to_dict(data):
    if isinstance(data, dict):
        return {snake_to_camle(k): model_to_dict(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [model_to_dict(v) for v in data]
    else:
        if dataclasses.is_dataclass(data):
            data_class_name = type(data).__name__
            
            if CLASS_MAP.get(data_class_name):
                data = dataclasses.asdict(data)
                data = props_to_camel(data)
                data["__name___"] = data_class_name
                
        return data
    

def dict_to_model(data):
    if isinstance(data, dict):
        data_class_name = data.get("__name___")
        data_class = CLASS_MAP.get(data_class_name)
        if data_class:
            data = {camel_to_snake(k): dict_to_model(v) for k, v in data.items()}
            try:
                d = from_dict(data_class, data)
            except Exception as e:
                print(e, data)
            return d
        else:
            return {camel_to_snake(k): dict_to_model(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [dict_to_model(v) for v in data]
    else:
        return data