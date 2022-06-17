from curses import nonl
import threading
from time import time
from typing import Callable, Coroutine, Optional, cast, Union
from typing import List, Dict, Set
from typing import TYPE_CHECKING

import asyncio
import functools
import importlib
import importlib.util
import concurrent.futures
import concurrent.futures.process
import multiprocessing
from multiprocessing import JoinableQueue
from multiprocessing.connection import Connection

import launch.logging

from ..db.web import ROS2WebDB
from ..db.web_package import DB
from ..api.web_package import WebPackage
from ..api.ros2 import ROS2API
from .ros_executor import ROSExecutor
from .wp_api import WebPackageAPI

from ..utilities.converter import props_to_camel, props_to_snake

if TYPE_CHECKING:
    from ..launch.actions import SystemService  # noqa: F401


def process_worker(*, name: str, class_name: str, module_name: str, location: str,
                   receive_queue: JoinableQueue, send_queue: JoinableQueue,
                   api_conn: Connection, service_conn: Connection):
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        module = importlib.import_module(module_name, location)
        web_package: WebPackage = getattr(module, class_name)()
        is_running = True
        logger = launch.logging.get_logger(f'process_worker[{name}]')
        
        def receive_data(queue: JoinableQueue, api: WebPackageAPI):
            nonlocal is_running
            while is_running:
                try:
                    data = queue.get()
                    api.set_receive_data(data)
                finally:
                    if queue.qsize() > 0:
                        queue.task_done()

        def call_api(conn: Connection, api: WebPackageAPI):
            nonlocal logger
            nonlocal is_running
            while is_running:
                try:
                    conn.send(api.call(conn.recv()))
                except ValueError as e:
                    logger.error(e)
                except EOFError:
                    pass

        def exception_handler(self, loop, context):
            nonlocal logger
            logger.error(context)
        
        async def catch_exception(awaitable):
            nonlocal logger
            try:
                return await awaitable
            except Exception as e:
                logger.error("startup: ".format(e))
                
        
        loop.set_exception_handler(exception_handler)
        
        ros_executor = ROSExecutor(node_name=f"ros2web_{name}",
                                   namespace=None,
                                   args=None,
                                   loop=loop)
        ros2_api = ROS2API(ros_node=ros_executor.node,
                           connection=service_conn, loop=loop)
        api = WebPackageAPI(name=name,
                            ros2_api=ros2_api,
                            send_queue=send_queue,
                            loop=loop)
        
        web_package._set_api(api)
        loop.run_in_executor(
            None, receive_data, receive_queue, api)
        loop.run_in_executor(
            None, call_api, api_conn, api)
        loop.create_task(catch_exception(web_package.on_startup()))
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        loop.run_until_complete(web_package.on_shutdown())
        ros_executor.shutdown()
        loop.close()


class WebPackageProcess:
    def __init__(self, *, name, class_name, module_name, location, receive_queue, service: 'SystemService'):
        self.__process = None
        self.__name = name
        self.__class_name = class_name
        self.__module_name = module_name
        self.__location = location
        self.__send_queue = JoinableQueue()
        self.__receive_queue = receive_queue
        self.__parent_api_conn = None
        self.__parent_service_conn = None
        self.__service = service
        self.__is_running = False

        self.__call_lock = threading.Lock()

        self.__loop = asyncio.get_event_loop()
        self.__logger = launch.logging.get_logger(f'process[{self.__name}]')

    def start(self):
        self.__is_running = True
        parent_service_conn, child_service_conn = multiprocessing.Pipe()
        parent_api_conn, child_api_conn = multiprocessing.Pipe()

        self.__parent_api_conn = parent_api_conn
        self.__parent_service_conn = parent_service_conn

        self.__loop.run_in_executor(None, self.__service_loop)

        self.__process = multiprocessing.Process(
            target=process_worker, kwargs={
                'name': self.__name,
                'class_name': self.__class_name,
                'module_name': self.__module_name,
                'location': self.__location,
                'receive_queue': self.__send_queue,
                'send_queue': self.__receive_queue,
                'api_conn': child_api_conn,
                'service_conn': child_service_conn,
            })
        self.__process.start()

    def shutdown(self):
        self.__is_running = False
        self.__process.terminate()
        self.__process.join()
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
        kwargs['wp_name'] = self.__name

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
                self.__logger.error(e)
            except EOFError:
                pass
            except Exception as e:
                self.__logger.error(e)

    def send_data(self, data):
        self.__send_queue.put(data)

    def __call_api(self, request):
        response = None
        try:
            with self.__call_lock:
                self.__parent_api_conn.send(request)
                if self.__parent_api_conn.poll(timeout=3.5):
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

    async def get_state(self):
        request = {
            'web_api_method': 'get_state'
        }
        return await self.__loop.run_in_executor(None, self.__call_api, request)
