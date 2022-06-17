from typing import Optional, Union, Iterable, Callable, cast
from typing import List, Dict, Set

import asyncio
from xmlrpc.client import boolean
import launch.logging
import threading
import time
from collections import defaultdict

from rclpy.node import Node as ROSNode
from rclpy.parameter import Parameter

from launch.actions import OpaqueFunction
from launch.events import matches_action
from launch.events.process import ProcessStarted, ProcessExited
from launch.events.process import ProcessStdout, ProcessStderr, ProcessStdin
from launch.events.process import ShutdownProcess
from launch.event_handler import EventHandler
from launch.launch_context import LaunchContext
from launch.utilities import is_a_subclass
from launch.some_substitutions_type import SomeSubstitutionsType
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules

# from launch.actions import ExecuteProcess
# from launch_ros.actions import Node
from .node_wrapper import Node
from .process import Process, ProcessStatus
from ..events import ExecutionProcess


class SystemService(OpaqueFunction):

    def __init__(self) -> None:
        super().__init__(function=self.__function)
        self.__processes: Set[Process] = set()
        self.__launch_context = None
        self.__lock = threading.Lock()
        self.__method_lock = threading.Lock()
        self.__send_queue = None
        self.__loop = asyncio.get_event_loop()
        self.__logger = launch.logging.get_logger('SystemService')

    def __function(self, context: LaunchContext):
        self.__launch_context = context

        event_handlers = [
            EventHandler(
                matcher=lambda event: is_a_subclass(event, ExecutionProcess),
                entities=OpaqueFunction(function=self.__execute_node),
            )
        ]
        for event_handler in event_handlers:
            context.register_event_handler(event_handler)

    def __execute_node(self, context: LaunchContext):
        process = cast(ExecutionProcess, context.locals.event).process
        if process.action is None:
            return []

        return [process.action]

    def __on_start(self, process: Process, event: ProcessStarted):
        with self.__lock:
            self.__processes.add(process)

        if self.__send_queue is not None:
            req = {
                'operation': 'system_event',
                'webPackageName': process.wp_name,
                'event': {
                    'id': process.id,
                    'pid': process.pid,
                    'event_type': 'on_start'
                }
            }
            asyncio.run_coroutine_threadsafe(
                self.__send_queue.put(req), self.__loop)

    def __on_exit(self, process: Process, event: ProcessExited):
        with self.__lock:
            self.__processes.remove(process)

        if self.__send_queue is not None:
            req = {
                'operation': 'system_event',
                'webPackageName': process.wp_name,
                'event': {
                    'id': process.id,
                    'pid': process.pid,
                    'event_type': 'on_exit'
                }
            }
            asyncio.run_coroutine_threadsafe(
                self.__send_queue.put(req), self.__loop)

    def __on_stdout(self, process: Process, event: ProcessStdout):
        if self.__send_queue is not None:
            if process.is_emit('on_stdout'):
                req = {
                    'operation': 'system_event',
                    'webPackageName': process.wp_name,
                    'event': {
                        'id': process.id,
                        'pid': process.pid,
                        'event_type': 'on_stdout',
                        'text': event.text
                    }
                }
                asyncio.run_coroutine_threadsafe(
                    self.__send_queue.put(req), self.__loop)

    def __on_stderr(self, process: Process, event: ProcessStderr):
        if self.__send_queue is not None:
            if process.is_emit('on_stderr'):
                req = {
                    'operation': 'system_event',
                    'webPackageName': process.wp_name,
                    'event': {
                        'id': process.id,
                        'pid': process.pid,
                        'event_type': 'on_stderr',
                        'text': event.text
                    }
                }
                asyncio.run_coroutine_threadsafe(
                    self.__send_queue.put(req), self.__loop)

    def set_send_queue(self, queue: asyncio.Queue):
        self.__send_queue = queue

    # API
    async def ros2_run(self, *,
                       req_id: str,
                       executable: SomeSubstitutionsType,
                       package: Optional[SomeSubstitutionsType] = None,
                       name: Optional[SomeSubstitutionsType] = None,
                       namespace: Optional[SomeSubstitutionsType] = None,
                       exec_name: Optional[SomeSubstitutionsType] = None,
                       parameters: Optional[SomeParameters] = None,
                       remappings: Optional[SomeRemapRules] = None,
                       arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
                       events: List[str] = [],
                       wp_name: str
                       ) -> Optional[Dict]:

        action = Node(executable=executable,
                      package=package,
                      name=name,
                      namespace=namespace,
                      exec_name=exec_name,
                      parameters=parameters,
                      remappings=remappings,
                      arguments=arguments)

        completed_future = self.__loop.create_future()
        process = Process(
            req_id=req_id,
            action=action, completed_future=completed_future, wp_name=wp_name, events=events)
        process.on_start = self.__on_start
        process.on_exit = self.__on_exit
        process.on_stdout = self.__on_stdout
        process.on_stderr = self.__on_stderr

        response = None
        try:
            with self.__method_lock:
                for event_handler in process.event_handlers:
                    self.__launch_context.register_event_handler(event_handler)

                execution_process_event = ExecutionProcess(process=process)
                self.__launch_context.emit_event_sync(execution_process_event)
                await completed_future
                response = req_id
        except asyncio.CancelledError:
            self.__logger.error("CancelledError.")
            for event_handler in process.event_handlers:
                self.__launch_context.unregister_event_handler(event_handler)
            event = ShutdownProcess(
                process_matcher=matches_action(process.action))
            self.__launch_context.emit_event_sync(event)
        finally:
            return response

    async def shutdown_process(self, *, pid: int, wp_name: str) -> boolean:
        processes = [
            process for process in self.__processes if process.pid == pid]
        process = processes[0] if len(processes) == 1 else None
        if process is not None:
            process.shutdown()
            return True
        else:
            return False
