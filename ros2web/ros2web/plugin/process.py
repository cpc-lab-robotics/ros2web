from typing import Callable, Coroutine, Optional, cast, Union
from typing import List, Dict, Set
from typing import TYPE_CHECKING

from types import ModuleType
import threading
import asyncio
import importlib
import importlib.util
import concurrent.futures
import concurrent.futures.process
import multiprocessing
from multiprocessing import JoinableQueue
from multiprocessing.connection import Connection

import launch.logging
from ..api import ROS2API
from ..api import Plugin as PluginPcackage
from .ros_executor import ROSExecutor
from .api import PluginAPI
from ..utilities.converter import props_to_camel, props_to_snake
from ..utilities import open_yaml_file


if TYPE_CHECKING:
    from ..launch.actions import SystemService
    from ..launch.actions.plugin_manager import Plugin


def reload_module(module, package_name, reloaded):
    """Recursively reload modules."""
    importlib.reload(module)
    reloaded[module.__name__] = module
    for attribute_name in dir(module):
        attribute = getattr(module, attribute_name)
        if type(attribute) is ModuleType:
            if attribute.__name__.startswith(package_name) and attribute.__name__ not in reloaded:
                reload_module(attribute, package_name, reloaded)


def process_worker(*, plugin: 'Plugin',
                   receive_queue: JoinableQueue, send_queue: JoinableQueue,
                   api_conn: Connection, service_conn: Connection, reload: bool,
                   stop_event):

    logger = launch.logging.get_logger(f'process_worker[{plugin.name}]')

    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        module = importlib.import_module(plugin.module_name, plugin.location)

        if reload:
            reload_module(module, plugin.package_name, {})
            module = importlib.reload(module)

        plugin_package: PluginPcackage = getattr(
            module, plugin.class_name)(plugin.config)
        is_running = True

        def receive_data(queue: JoinableQueue, api: PluginAPI):
            nonlocal is_running
            while is_running:
                try:
                    data = queue.get()
                    api.set_receive_data(data)
                finally:
                    if queue.qsize() > 0:
                        queue.task_done()

        def send_data(payload: Dict):
            nonlocal send_queue
            nonlocal plugin

            event = {
                'operation': "update",
                'webPackageName': plugin.name,
                'payload': payload
            }
            send_queue.put(event)

        def call_api(conn: Connection, api: PluginAPI):
            nonlocal logger
            nonlocal is_running
            while is_running:
                try:
                    conn.send(api.call(conn.recv()))
                except ValueError as e:
                    logger.error(e)
                except EOFError:
                    pass

        call_service_lock = threading.Lock()
        def call_service(request):
            nonlocal service_conn
            nonlocal call_service_lock
            with call_service_lock:
                service_conn.send(request)
                if service_conn.poll(timeout=3.5):
                    response = service_conn.recv()
            return response

        def exception_handler(self, loop, context):
            nonlocal logger
            logger.error(context)

        async def catch_exception(awaitable):
            nonlocal logger
            try:
                return await awaitable
            except Exception as e:
                logger.error("startup: ".format(e))

        async def run_forever(event):
            while not event.is_set():
                await asyncio.sleep(1)

        loop.set_exception_handler(exception_handler)

        ros_executor = ROSExecutor(node_name=plugin.package_name,
                                   namespace=None,
                                   args=None,
                                   loop=loop)
        ros2_api = ROS2API(ros_node=ros_executor.node,
                           sys_service=call_service, loop=loop)

        send_data = send_data if plugin.type == 'package' else None
        api = PluginAPI(ros2_api=ros2_api, send_data=send_data, loop=loop)
        plugin_package._set_api(api)
        plugin_package._set_sys_service(call_service)

        loop.run_in_executor(
            None, receive_data, receive_queue, api)
        loop.run_in_executor(
            None, call_api, api_conn, api)
        loop.create_task(catch_exception(plugin_package.on_startup()))

        # loop.run_forever()
        loop.run_until_complete(run_forever(stop_event))
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        loop.run_until_complete(plugin_package.on_shutdown())
        ros_executor.shutdown()
        loop.close()


class PluginProcess:
    def __init__(self, *, plugin: 'Plugin',
                 receive_queue: JoinableQueue, service: 'SystemService'):

        self.__plugin = plugin
        self.__receive_queue = receive_queue
        self.__service = service

        self.__send_queue = None
        self.__parent_api_conn = None
        self.__parent_service_conn = None
        self.__is_running = False
        self.__process = None
        self.__reload = False
        self.__stop_event = None
        self.__call_lock = threading.Lock()
        self.__loop = asyncio.get_event_loop()
        self.__logger = launch.logging.get_logger(
            f'process[{self.__plugin.name}]')

    def restart(self, plugin):
        self.__plugin = plugin
        
        if self.__is_running:
            with self.__call_lock:
                self.shutdown()
                self.start()

    def start(self):
        process_disable = self.__plugin.config.get("process_disable", False)
        if process_disable is True:
            return
        
        if self.__is_running is False:
            self.__is_running = True

            parent_service_conn, child_service_conn = multiprocessing.Pipe()
            parent_api_conn, child_api_conn = multiprocessing.Pipe()

            self.__send_queue = JoinableQueue()
            self.__parent_api_conn = parent_api_conn
            self.__parent_service_conn = parent_service_conn
            
            self.__loop.run_in_executor(None, self.__service_loop)

            self.__stop_event = multiprocessing.Event()

            # TODO: Rewrite process_worker as process class.
            self.__process = multiprocessing.Process(
                target=process_worker, kwargs={
                    'plugin': self.__plugin,
                    'receive_queue': self.__send_queue,
                    'send_queue': self.__receive_queue,
                    'api_conn': child_api_conn,
                    'service_conn': child_service_conn,
                    'reload': self.__reload,
                    'stop_event': self.__stop_event
                })
            
            self.__process.start()
            self.__reload = True

    def shutdown(self):
        if self.__process is None:
            return
        
        if self.__is_running:
            self.__is_running = False
            if self.__process.is_alive():
                self.__stop_event.set()
                self.__process.join(timeout=3.0)
            self.__process.terminate()
                
            self.__send_queue.close()
            self.__send_queue.join_thread()
            self.__parent_service_conn.close()
            self.__parent_api_conn.close()

    def __call_service(self, data, timeout: float = 3.0):
        method_name: str = data.get('method_name')
        if method_name.startswith("_"):
            return None

        args = data.get('args', [])
        kwargs = data.get('kwargs', {})
        kwargs['plugin_id'] = self.__plugin.id

        response = None
        try:
            future = asyncio.run_coroutine_threadsafe(
                getattr(self.__service, method_name)(*args, **kwargs),
                self.__loop)
            response = future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            self.__logger.error(
                'The coroutine took too long, cancelling the task...')
            future.cancel()
        except Exception as e:
            self.__logger.error('call_service[{}]: {}'.format(method_name, e))

        return response

    def __service_loop(self):
        while self.__is_running:
            try:
                self.__parent_service_conn.send(
                    self.__call_service(self.__parent_service_conn.recv())
                )
            except ValueError as e:
                self.__logger.error('service_loop: '.format(e))
            except EOFError:
                pass
            except Exception as e:
                self.__logger.error('service_loop: '.format(e))

    def send_data(self, data):
        self.__send_queue.put(data)

    def __call_api(self, request):
        response = None
        try:
            with self.__call_lock:
                self.__parent_api_conn.send(request)
                if self.__parent_api_conn.poll(timeout=5):
                    response = self.__parent_api_conn.recv()
        except EOFError as e:
            self.__logger.error("[call_api] EOFError: {}".format(e))
        except ValueError as e:
            self.__logger.error("[call_api] ValueError: {}".format(e))
        return response

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
        return await self.__loop.run_in_executor(None, self.__call_api, request)

    # async def get_state(self):
    #     request = {
    #         'api_method': 'get_state'
    #     }
    #     return await self.__loop.run_in_executor(None, self.__call_api, request)
