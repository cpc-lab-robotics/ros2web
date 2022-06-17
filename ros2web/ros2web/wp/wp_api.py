from typing import Any, Dict, List, Set
from typing import Optional, Union, Iterable, Callable, NamedTuple
from typing import TYPE_CHECKING

import asyncio
from asyncio import AbstractEventLoop
import threading
from uuid import uuid4
import concurrent.futures
from multiprocessing import JoinableQueue
from collections import defaultdict

import launch.logging
from rclpy.node import Node as ROSNode

from ..api.event import WidgetEvent
from ..api.ros2 import ROS2API
from ..api.route_table_def import RouteTableDef, Request
from ..api.models import model_to_dict, dict_to_model, dataclasses_to_dict


if TYPE_CHECKING:
    from ..api.web_package import WebPackage


class WebPackageAPIException(Exception):
    pass


class WebPackageAPI:

    def __init__(self, *,
                 name, ros2_api: ROS2API,
                 send_queue: JoinableQueue,
                 loop: AbstractEventLoop
                 ) -> None:

        self.__bind = defaultdict(set)
        self.__handlers: Dict[str, Callable] = {}
        self.__ui_element = None
        self.__lock = threading.Lock()
        self.__reload = False
        self.__xml = None
        self.__routes: RouteTableDef = None
        self.__name = name
        self.__ros2_api = ros2_api
        self.__web_package = None
        self.__send_queue = send_queue
        self.__loop = loop
        self.__logger = launch.logging.get_logger(f"{name}:api")

    def __set_state(self, data: Dict):
        with self.__lock:
            for k, v in data.items():
                self.__state[k] = v

    def init(self, *, web_package: 'WebPackage',
             state: Dict,
             routes: Optional[RouteTableDef] = None):

        self.__web_package = web_package
        self.__state = state
        self.__routes = routes

    def set_receive_data(self, data, timeout: float = 3.0) -> None:
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
                    future = asyncio.run_coroutine_threadsafe(
                        handler(new_widget_event), self.__loop)
                    future.result(timeout=timeout)
                except concurrent.futures.TimeoutError:
                    self.__logger.error(
                        'The coroutine took too long, cancelling the task...')
                    future.cancel()
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

        elif operation == 'system_event':
            if self.__ros2_api is None:
                return
            event = data.get('event', {})
            self.__ros2_api._emit_event(event)

    def call(self, data, timeout: float = 3.0):
        response = None
        web_api_method: str = data.get('web_api_method')
        if web_api_method is not None:
            kwargs: Dict = data.get('kwargs', {})
            try:
                method = getattr(self, web_api_method, None)
                if method is None:
                    return None
                response = method(**kwargs)
            except Exception as e:
                self.__logger.error("call[{}]: {}".format(web_api_method, e))
            return response
        else:
            if self.__routes is None:
                return None
            request_method: str = data.get('request_method')
            path: str = data.get('path')
            search_params: Dict = data.get('search_params', {})
            json_payload: Dict = data.get('json_payload', {})

            method_name, params = self.__routes.get_method_name(
                request_method=request_method,
                path=path,
            )
            request = Request(
                match_params=params,
                search_params=search_params,
                json_payload=json_payload
            )

            method = getattr(self.__web_package, method_name, None)
            if method is None:
                return None
            try:
                future = asyncio.run_coroutine_threadsafe(
                    getattr(self.__web_package, method_name)(request), self.__loop)
                response = future.result(timeout=timeout)
                if request_method == 'page':
                    response = self.__init_page(response)
                else:
                    response = dataclasses_to_dict(response)
            except concurrent.futures.TimeoutError:
                self.__logger.error('TimeoutError')
                future.cancel()
            except Exception as e:
                self.__logger.error('call_api[{}]: {}'.format(method_name, e))
            return response

    def get_state(self) -> Dict:
        _state = {}
        with self.__lock:
            for key, value in self.__state.items():
                _state[key] = model_to_dict(value)
        return _state

    def __add_key_widget(self, widgets: List):
        for widget in widgets:
            if isinstance(widget, dict):
                items = list(widget.values())
                if len(items) > 0:
                    widget_el = items[0]
                    if isinstance(widget_el, dict):
                        widget_el['key'] = str(uuid4())
                        _widgets = widget_el.get('widgets', [])
                        self.__add_key_widget(_widgets)

    def __add_key(self, settings: Dict):
        if isinstance(settings, dict):
            layout = settings.get('layout')
            if isinstance(layout, dict):
                items = list(layout.values())
                if len(items) > 0:
                    layout_el = items[0]
                    if isinstance(layout_el, dict):
                        layout_el['key'] = str(uuid4())
            widgets = settings.get('widgets', [])
            self.__add_key_widget(widgets)

    def __init_page(self, ui: Dict):
        bind = {}
        for k, v in self.__bind.items():
            bind[k] = list(v)
        self.__add_key(ui)

        return {
            'ui': ui,
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
        _state = {}

        for key, value in state.items():
            _state[key] = model_to_dict(value)

        event = {
            'operation': "update",
            'webPackageName': self.__name,
            'payload': _state
        }
        self.__send_queue.put(event)
        self.__set_state(state)

    @property
    def state(self) -> Dict:
        return self.__state

    @property
    def name(self) -> str:
        return self.__name

    @property
    def ros2(self) -> ROS2API:
        return self.__ros2_api

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        return self.__loop
