from typing import Optional, Union, Iterable, Callable, cast, TYPE_CHECKING
from typing import List, Dict, Set, Any

import asyncio
import launch.logging
import threading
from multiprocessing import JoinableQueue
from multiprocessing.connection import Connection

from launch.actions import OpaqueCoroutine, OpaqueFunction
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

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from .process import Process, ProcessStatus
from ..events import ExecutionProcess


class ProcessManager(OpaqueCoroutine):
    
    def __init__(self) -> None:
        super().__init__(coroutine=self.__coroutine)
        self.__processes: Set[Process] = set()
        self.__launch_context = None
        self.__loop = None
        
        self.__lock = threading.Lock()
        self.__logger = launch.logging.get_logger('ProcessManager')

    async def __coroutine(self, context: LaunchContext):
        self.__launch_context = context
        self.__loop = self.__launch_context.asyncio_loop

        event_handlers = [
            EventHandler(
                matcher=lambda event: is_a_subclass(event, ExecutionProcess),
                entities=OpaqueFunction(function=self.__execute_node),
            )
        ]
        for event_handler in event_handlers:
            context.register_event_handler(event_handler)
        try:
            await self.__loop.create_future()
        except asyncio.CancelledError:
            for process in self.__processes:
                process.terminate()

    def __execute_node(self, context: LaunchContext):
        process = cast(ExecutionProcess, context.locals.event).process
        if process.action is None:
            return []
        
        with self.__lock:
            self.__processes.add(process)
        
        return [process.action]

    def __on_exit(self, process: Process, event: ProcessExited):
        with self.__lock:
            self.__processes.remove(process)

    async def execute_process(self, *, action: Union[ExecuteProcess, Node], connection: Connection):
        
        process = Process(action=action, connection=connection, loop=self.__loop)
        process.on_exit = self.__on_exit
        
        for event_handler in process.event_handlers:
            self.__launch_context.register_event_handler(event_handler)
        execution_process_event = ExecutionProcess(process=process)
        self.__launch_context.emit_event_sync(execution_process_event)
        