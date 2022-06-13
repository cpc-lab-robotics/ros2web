from typing import Optional, Tuple
from typing import List, Dict, Any
import functools
import re
from dataclasses import dataclass, field
from collections import defaultdict


@dataclass
class Request:
    match_params: Dict[str, Any] = field(default_factory=dict)
    search_params: Dict[str, Any] = field(default_factory=dict)
    json_payload: Dict[str, str] = field(default_factory=dict)


class RouteTableDef:
    def __init__(self) -> None:
        self.__routes: Dict[str, List] = defaultdict(list)

    @property
    def page(self):
        def _page(method):
            self.__routes['page'].append({
                'method_name': method.__name__,
            })

            @functools.wraps(method)
            async def wrapper(*args, **kwargs):
                return await method(*args, **kwargs)
            return wrapper
        return _page

    def page_get(self, path: str):
        path = path[1:] if path.startswith("/") else path
        regex = re.sub(r"{(\S+?)}", r"(?P<\1>\\S+)", path)

        def _page(method):
            self.__routes['page'].append({
                'method_name': method.__name__,
                'regex': regex
            })

            @functools.wraps(method)
            async def wrapper(*args, **kwargs):
                return await method(*args, **kwargs)
            return wrapper
        return _page

    def get(self, path: str):
        path = path[1:] if path.startswith("/") else path
        regex = re.sub(r"{(\S+?)}", r"(?P<\1>\\S+)", path)

        def _get(method):
            self.__routes['get'].append({
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
            self.__routes['post'].append({
                'method_name': method.__name__,
                'regex': regex
            })

            @functools.wraps(method)
            async def wrapper(*args, **kwargs):
                return await method(*args, **kwargs)
            return wrapper
        return _post

    def get_method_name(self, *, request_method: str, path: str) -> Tuple[str, Dict]:
        params = {}
        method_name = ""
        for api in self.__routes[request_method]:
            regex = api.get('regex', None)
            if regex is None:
                method_name = api.get("method_name", "")
                break
            else:
                m = re.fullmatch(regex, path)
                if m is not None:
                    params = m.groupdict()
                    method_name = api.get("method_name", "")
                    break

        return method_name, params
