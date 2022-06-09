from typing import List, Dict, Tuple
import functools
import re

http_request_method = ['get', 'post']
# http_request_method = ['get', 'post', 'put', 'delete']

class APIDefinition:
    def __init__(self) -> None:
        self.__api_dict:Dict[str, List] = {}
        for method in http_request_method:
            self.__api_dict[method] = []
    
    def get(self, path: str):
        path = path[1:] if path.startswith("/") else path
        regex = re.sub(r"{(\S+?)}", r"(?P<\1>\\S+)", path)
        
        def _get(method):
            self.__api_dict['get'].append({
                'method_name': method.__name__,
                'regex': regex
            })
            @functools.wraps(method)
            async def wrapper(*args, **kwargs):
                return await method(*args, **kwargs)
            return wrapper
        return _get

    def post(self, path: str):
        path = path[1:] if path.startswith("/") else path
        regex = re.sub(r"{(\S+?)}", r"(?P<\1>\\S+)", path)
        
        def _post(method):
            self.__api_dict['post'].append({
                'method_name': method.__name__,
                'regex': regex
            })
            @functools.wraps(method)
            async def wrapper(*args, **kwargs):
                return await method(*args, **kwargs)
            return wrapper
        return _post

    def get_method_name(self, *, request_method:str, path:str) -> Tuple[str, Dict]:
        params = {}
        method_name = ""
        for api in self.__api_dict[request_method]:
            m = re.fullmatch(api['regex'], path)
            if m is not None:
                params = m.groupdict()
                method_name = api.get("method_name", "")
                break
        return method_name, params
