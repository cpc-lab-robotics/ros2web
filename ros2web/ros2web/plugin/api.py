from typing import Any, Dict, List, Set
from typing import Optional, Union, Iterable, Callable, NamedTuple
from typing import TYPE_CHECKING

import asyncio
from asyncio import AbstractEventLoop
import threading
import concurrent.futures
from collections import defaultdict
from collections import namedtuple

import launch.logging
from rclpy.node import Node as ROSNode

from ..api.event import WidgetEvent
from ..api.ros2 import ROS2API
from ..api.route_table_def import RouteTableDef, Request
from ..api.models import model_to_dict, dict_to_model, dataclasses_to_dict
from ..utilities import extract_state_keys, UniqueName

if TYPE_CHECKING:
    from ..api.plugin import Plugin

class PluginAPIException(Exception):
    pass


class PluginAPI:

    def __init__(self, *,
                 send_data: Optional[Callable]=None,
                 loop: AbstractEventLoop
                 ) -> None:
        
        self.__logger = launch.logging.get_logger("PluginAPI")
        self.__bind = defaultdict(set)
        self.__handlers: Dict[str, Callable] = {}
        self.__lock = threading.Lock()
        
        self.__send_data = send_data
        self.__loop = loop

        self.__plugin: 'Plugin' = None
        self.__routes: RouteTableDef = None
        self.__state: Dict = {}

        self.__state_key: Set = set()

    def __set_state(self, data: Dict):
        with self.__lock:
            for k, v in data.items():
                if k in self.__state_key:
                    self.__state[k] = v

    def init(self, *, plugin: 'Plugin',
             state: Dict,
             routes: Optional[RouteTableDef] = None):

        self.__plugin = plugin
        
        # self.__state_class = namedtuple('State', state.keys())
        # self.__state = self.__state_class(**state)
        
        self.__state = state
        self.__state_key = set(state.keys())
        self.__routes = routes

    async def set_receive_data(self, data) -> None:
        operation = data.get('operation')
        if operation == 'emit':
            widget_event = data.get('widgetEvent')
            event = widget_event.get('event')
            widget_id = event.get("widget_id")
            event_type = event.get("type")
            event_id = f"{widget_id}:{event_type}"
            value = widget_event.get('value')
            model_value = dict_to_model(value)

            new_widget_event = WidgetEvent(widget_id, event_type, model_value)
            
            handler = self.__handlers.get(event_id)
            if handler is not None:
                try:
                    await handler(new_widget_event)
                except Exception as e:
                    self.__logger.error(
                        'event_handler[{}]: {}'.format(handler.__name__, e))

        elif operation == 'update':
            try:
                payload = data.get('payload', {})
                _state = {}
                for key, value in payload.items():
                    _state[key] = dict_to_model(value)
                self.__set_state(_state)
            except Exception as e:
                self.__logger.error("update error: ".format(e))
            

    async def call(self, req):
        
        request_method: str = req.get('request_method')
        path: str = req.get('path')
        search_params: Dict = req.get('search_params', {})
        json_payload: Dict = req.get('json_payload', {})

        method_name, params = self.__routes.get_method_name(
            request_method=request_method,
            path=path,
        )
        
        request = Request(
            match_params=params,
            search_params=search_params,
            json_payload=json_payload
        )
        
        response = None
        method = getattr(self.__plugin, method_name, None)
        if method is None:
            return None
        try:
            response = await method(request)
            if request_method == 'page':
                response = self.__init_page(response)
            else:
                response = dataclasses_to_dict(response)
        except Exception as e:
            self.__logger.error('call_api[{}]: {}'.format(method_name, e))
        return response
            

    def get_state(self) -> Dict:
        _state = {}
        with self.__lock:
            for key, value in self.__state.items():
                _state[key] = model_to_dict(value)
        return _state

    def __init_page(self, page: Dict):
        bind = {}
        for k, v in self.__bind.items():
            bind[k] = list(v)
        
        unique_name = UniqueName()
        widgets = page.get('widgets', [])
        widgets = [] if widgets is None else widgets
        
        for widget in widgets:
            widget_name: str = widget.get('name')
            if widget_name is None:
                raise RuntimeError("No widget name")

            widget['key'] = unique_name.get_name(widget_name.lower())
            props = widget.get('props', {})
            state_keys = set()
            extract_state_keys(props, state_keys)
            init_state = {}
            for key in state_keys:
                if key in self.__state_key:
                    value = self.__state[key]
                    init_state[key] = model_to_dict(value)
                else:
                    raise KeyError(f"'{key}' is not declared in 'init_state'.")
            widget['init_state'] = init_state
        return {
            'style': page.get('style'),
            'widgets': widgets,
            'bind': bind
        }

    def bind(self, widget_id, event_type, handler):
        event_id = f"{widget_id}:{event_type}"
        self.__handlers[event_id] = handler
        self.__bind[widget_id].add(event_id)

    def unbind(self, widget_id, event_type):
        event_id = f"{widget_id}:{event_type}"
        del self.__handlers[event_id]
        self.__bind[widget_id].remove(event_id)

    def set_state(self, state: Dict):
        if self.__send_data:
            _state = {}
            for key, value in state.items():
                if key in self.__state_key:
                    _state[key] = model_to_dict(value)
            self.__set_state(state)
            self.__send_data(_state)

    @property
    def state(self) -> Dict:
        return self.__state

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        return self.__loop
