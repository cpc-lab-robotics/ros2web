from typing import Callable, Coroutine, Optional, cast, Union
from typing import List, Dict, Set
from typing import TYPE_CHECKING

import time
from types import ModuleType
import threading
import asyncio
import importlib
import importlib.util
import concurrent.futures
import concurrent.futures.process
import multiprocessing as mp
from multiprocessing import JoinableQueue
from multiprocessing.connection import Connection

import launch.logging

from ..api.ros2 import ROS2API
from ..api import Plugin as PluginPcackage
from .ros_executor import ROSExecutor
from .api import PluginAPI
from ..utilities.converter import props_to_camel, props_to_snake
from ..utilities import open_yaml_file, reload_module
from ..launch.service import DynamicLaunchService


if TYPE_CHECKING:
    from .manager import Plugin


class PluginProcess(mp.Process):
    def __init__(self, *, plugin: 'Plugin', receive_queue: JoinableQueue, loop: asyncio.AbstractEventLoop):
        super().__init__()

        self.__plugin = plugin
        self.__receive_queue = receive_queue
        parent_conn, child_conn = mp.Pipe()
        self.__parent_conn = parent_conn
        self.__child_conn = child_conn
        self.__call_lock = threading.Lock()

        self.__logger = launch.logging.get_logger(
            f'process[{self.__plugin.name}]')

        self.__parent_loop = loop
        self.__is_running = mp.Value('i', False)

    @property
    def is_running(self) -> bool:
        return bool(self.__is_running.value)

    async def call_api(self, *,
                       request_method: str,
                       path: str,
                       search_params: Optional[Dict] = None,
                       json_payload: Optional[Dict] = None):

        request = {
            'request_method': request_method,
            'path': path,
            'search_params': search_params,
            'json_payload': json_payload
        }
        kwargs = {'request': request}

        return await self.__parent_loop.run_in_executor(None, self.__call_method, '_api_call', [], kwargs)

    async def _api_call(self, *, request):
        return await self.__api.call(request)

    async def recv_data(self, data):
        await self.__parent_loop.run_in_executor(None, self.__call_method, '_recv_data', [data], {})

    async def _recv_data(self, data):
        await self.__api.set_receive_data(data)

    def _send_data(self, payload: Dict):
        event = {
            'operation': "update",
            'webPackageName': self.__plugin.name,
            'payload': payload
        }
        self.__receive_queue.put(event)

    def shutdown(self):
        time.sleep(0.1)
        if self.is_running == True:
            self.__call_method("_shutdown", [], {})

    async def _shutdown(self):
        self.__future.set_result(None)

    async def main(self):
        self.__future = asyncio.get_event_loop().create_future()
        try:
            await self.__plugin_package.on_startup()
            await self.__future
        except Exception as e:
            self.__logger.error("{}".format(e))

    def start(self):
        process_disable = self.__plugin.config.get("process_disable", False)
        if process_disable is False:
            super().start()

    def run(self):
        try:
            self.__is_running.value = True
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            self.__loop = loop
            self.__loop.run_in_executor(None, self.__method_loop)

            module = importlib.import_module(
                self.__plugin.module_name, self.__plugin.location)

            reload_module(module, self.__plugin.package_name, {})
            module = importlib.reload(module)

            self.__plugin_package: PluginPcackage = getattr(
                module, self.__plugin.class_name)(self.__plugin.config)

            ros_executor = ROSExecutor(node_name=self.__plugin.package_name,
                                       namespace=None,
                                       args=None,
                                       loop=self.__loop)

            launch_service = DynamicLaunchService()

            ros2_api = ROS2API(ros_node=ros_executor.node,
                               launch_service=launch_service,
                               loop=self.__loop)

            send_data = self._send_data if self.__plugin.type == 'package' else None
            self.__api = PluginAPI(send_data=send_data, loop=self.__loop)
            self.__plugin_package._set_api(self.__api)
            self.__plugin_package._set_ros2api(ros2_api)
            launch_service.daemon = True
            launch_service.start()
            self.__loop.run_until_complete(self.main())
        except KeyboardInterrupt:
            pass
        finally:
            self.__is_running.value = False
            self.__loop.run_until_complete(self.__plugin_package.on_shutdown())
            launch_service.shutdown()
            ros_executor.shutdown()
            
            async def waiting():
                counter = 0
                while launch_service.is_running and counter < 10:
                    counter += 1
                    await asyncio.sleep(0.2)
            
            self.__loop.run_until_complete(waiting())
            self.__loop.close()

    def __call_method(self, method_name, args, kwargs):
        if self.is_running is False:
            return None

        request = {
            'method_name': method_name,
            'args': args,
            'kwargs': kwargs
        }
        response = None
        try:
            with self.__call_lock:
                self.__parent_conn.send(request)
                if self.__parent_conn.poll(timeout=3.5):
                    response = self.__parent_conn.recv()
        except EOFError as e:
            self.__logger.error("[call_api] EOFError: {}".format(e))
        except ValueError as e:
            self.__logger.error("[call_api] ValueError: {}".format(e))
        except BrokenPipeError as e:
            self.__logger.error("[call_api] BrokenPipeError: {}".format(e))

        return response

    def __call(self, data, timeout: float = 3.0):
        try:
            method_name: str = data.get('method_name')
            args = data.get('args', [])
            kwargs = data.get('kwargs', {})
            response = None

            if self.__loop.is_closed() == False:
                future = asyncio.run_coroutine_threadsafe(
                    getattr(self, method_name)(*args, **kwargs), self.__loop)
                response = future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            self.__logger.error(
                'The coroutine took too long, cancelling the task...')
            future.cancel()
        except Exception as e:
            self.__logger.error('call_service[{}]: {}'.format(method_name, e))
        return response

    def __method_loop(self):
        while self.is_running:
            try:
                self.__child_conn.send(
                    self.__call(self.__child_conn.recv())
                )
            except ValueError as e:
                self.__logger.error('{}'.format(e))
            except EOFError:
                pass
            except Exception as e:
                self.__logger.error('{}'.format(e))
