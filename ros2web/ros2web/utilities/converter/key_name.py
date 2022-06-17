from ..re import snake_to_camle, camel_to_snake


def props_to_camel(data):
    if isinstance(data, dict):
        return {snake_to_camle(k): props_to_camel(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [props_to_camel(v) for v in data]
    else:
        return data


def props_to_snake(data):
    if isinstance(data, dict):
        return {camel_to_snake(k): props_to_snake(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [props_to_snake(v) for v in data]
    else:
        return data
