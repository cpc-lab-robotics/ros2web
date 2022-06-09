from typing import Dict, List, Set
from typing import Optional, Union, Iterable, Callable
from typing import TYPE_CHECKING

import asyncio
from asyncio import AbstractEventLoop
import threading
import ujson
import concurrent.futures
from multiprocessing import JoinableQueue
from collections import defaultdict

import launch.logging
from rclpy.node import Node as ROSNode

from ..api.ros2 import ROS2API
from ..api.api_def import APIDefinition
from ..api.models import model_to_dict, dict_to_model, dataclasses_to_dict
from ..utilities.ui import create_element

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
        self.__api_def: APIDefinition = None
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
            # self.__copy_state = ujson.loads(ujson.dumps(self.__state))

    def init(self, *, web_package: 'WebPackage', state: Dict,
             xml: Optional[str] = None,
             api_def: Optional[APIDefinition] = None):

        self.__web_package = web_package
        self.__state = state
        # self.__copy_state: Dict = ujson.loads(ujson.dumps(self.__state))
        self.__api_def = api_def
        self.load_xml(xml)

    def set_receive_data(self, data, timeout: float = 3.0) -> None:
        operation = data.get('operation')
        if operation == 'emit':
            widget_event = data.get('widgetEvent')
            event = widget_event.get('event')
            widget_id = event.get("widget_id")
            event_type = event.get("type")
            event_id = f"{widget_id}:{event_type}"
            value = widget_event.get('value')
            data = dict_to_model(value)
            handler = self.__handlers.get(event_id)
            if handler is not None:
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        handler(event, data), self.__loop)
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
        path: str = data.get('path')
        kwargs: Dict = data.get('params', {})
        response = None

        if path.startswith("_"):
            try:
                method_name = path[1:]
                method = getattr(self, method_name, None)
                if method is None:
                    return None
                response = method(**kwargs)
            except Exception as e:
                self.__logger.error("call_api[{}]: {}".format(method_name, e))

            return response
        else:
            if self.__api_def is None:
                return None
            request_method: str = data.get('request_method')
            method_name, params = self.__api_def.get_method_name(
                request_method=request_method,
                path=path,
            )
            kwargs = {**kwargs, **params}
            method = getattr(self.__web_package, method_name, None)

            if method is None:
                return None

            try:
                future = asyncio.run_coroutine_threadsafe(
                    getattr(self.__web_package, method_name)(**kwargs), self.__loop)
                response = future.result(timeout=timeout)
                response = dataclasses_to_dict(response)
            except concurrent.futures.TimeoutError:
                self.__logger.error(
                    'The coroutine took too long, cancelling the task...')
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

    def get_info(self):
        if self.__reload:
            self.load_xml(self.__xml, reload=self.__reload)

        bind = {}
        for k, v in self.__bind.items():
            bind[k] = list(v)

        info = {}
        info['ui'] = {
            'element': self.__ui_element,
            'bind': bind
        }
        return info

    def load_xml(self, xml, *, reload=True):
        self.__reload = reload
        self.__xml = xml
        try:
            self.__ui_element = create_element(name=self.__name, xml=xml)
        except Exception as e:
            self.__logger.error("{}".format(e))

    def set_api_def(self, api_def: APIDefinition):
        self.__api_def = api_def

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
