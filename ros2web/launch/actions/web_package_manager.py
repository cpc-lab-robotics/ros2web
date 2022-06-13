from typing import Optional, cast
from typing import List, Dict, Set

import asyncio
import pkg_resources
from multiprocessing import JoinableQueue

import launch.logging
from launch.actions import OpaqueFunction, OpaqueCoroutine
from launch.event import Event
from launch.event_handler import EventHandler
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from launch.utilities import is_a_subclass

from ..events import ExecutionWebPackage
from .system_service import SystemService
from ...wp.wp_process import WebPackageProcess
from ...extensions import extensions


class WebPackageManager(OpaqueCoroutine):

    def __init__(self) -> None:
        super().__init__(coroutine=self.__coroutine)

        self.__logger = launch.logging.get_logger('WebPackageManager')
        
        self.__web_package_process_dict: Dict[str, WebPackageProcess] = {}
        self.__web_package_module_dict: Dict = {}
        self.__send_queue = None
        self.__receive_queue: asyncio.Queue = asyncio.Queue()
        self.__is_running = False
        self.__receive_wp_queue: JoinableQueue = JoinableQueue()
        self.__receive_loop_task = None
        self.__launch_context = None
        self.__loop = asyncio.get_event_loop()

    def __on_shutdown(self, event: Event, context: LaunchContext):
        self.__is_running = False
        self.__receive_wp_queue.put(None)

        if self.__receive_loop_task is not None:
            self.__receive_loop_task.cancel()

        for process in self.__web_package_process_dict.values():
            process.shutdown()

    async def __coroutine(self, context: LaunchContext):
        self.__launch_context = context

        event_handlers = [
            OnShutdown(on_shutdown=self.__on_shutdown),
            EventHandler(
                matcher=lambda event: is_a_subclass(
                    event, ExecutionWebPackage),
                entities=OpaqueFunction(
                    function=lambda context: [
                        cast(ExecutionWebPackage, context.locals.event).web_package]
                ),
            )
        ]
        for event_handler in event_handlers:
            context.register_event_handler(event_handler)

        self.__is_running = True
        self.__loop.run_in_executor(None, self.__receive_wp_queue_loop)
        self.__receive_loop_task = self.__loop.create_task(self.__receive_loop())

        self.__load_entry_points()
        self.__activation_web_packages()

    def __load_entry_points(self):
        
        for extension in extensions:
            self.__web_package_module_dict[extension.name] = {
                'name': extension.name,
                'class_name': extension.class_name,
                'module_name': extension.module_name,
                'location': extension.location
            }
            
        for entry_point in pkg_resources.iter_entry_points('ros2web.package'):
            try:
                class_name = entry_point.attrs[0]
                self.__web_package_module_dict[entry_point.name] = {
                    'name': entry_point.name,
                    'class_name': class_name,
                    'module_name': entry_point.module_name,
                    'location': entry_point.dist.location
                }
            except Exception as e:
                self.__logger.error("[{}]:{}".format(entry_point.name, e))

    def __activation_web_packages(self):
        # TODO: Only the Web Package that has been enabled is processed.
        
        service: SystemService = self.__launch_context.\
            get_locals_as_dict().get("system_service")

        service.set_send_queue(self.__receive_queue)

        for name, kwargs in self.__web_package_module_dict.items():
            try:
                process = WebPackageProcess(service=service, receive_queue=self.__receive_wp_queue, **kwargs)
                self.__web_package_process_dict[name] = process
                process.start()
            except Exception as e:
                self.__logger.error(
                    "Failed to activate '{}'. ({})".format(name, e))

    async def __hot_reload_loop(self):
        pass

    def __receive_wp_queue_loop(self):
        try:
            while self.__is_running:
                try:
                    data = self.__receive_wp_queue.get()
                    if self.__send_queue is not None and self.__is_running:
                        # self.__logger.info("send queue:{}".format(data))
                        asyncio.run_coroutine_threadsafe(
                            self.__send_queue.put(data), self.__loop)
                except Exception as e:
                    self.__logger.error(e)
                finally:
                    if self.__receive_wp_queue.qsize() > 0:
                        self.__receive_wp_queue.task_done()
        finally:
            self.__receive_wp_queue.close()
            self.__receive_wp_queue.join_thread()

    async def __receive_loop(self):
        try:
            while True:
                try:
                    data = await self.__receive_queue.get()
                    web_package_name = data.get('webPackageName')
                    process = self.__web_package_process_dict.get(web_package_name)
                    if process is not None:
                        process.send_data(data)
                except Exception as e:
                    self.__logger.error(e)
                finally:
                    if self.__receive_queue.qsize() > 0:
                        self.__receive_queue.task_done()
        except asyncio.CancelledError:
            pass
        finally:
            pass
    
    @property
    def receive_queue(self):
        return self.__receive_queue
    
    def set_send_queue(self, queue: asyncio.Queue):
        self.__send_queue = queue

    def get_web_package_process(self, web_package_name: str):
        process = self.__web_package_process_dict.get(web_package_name)
        return process
