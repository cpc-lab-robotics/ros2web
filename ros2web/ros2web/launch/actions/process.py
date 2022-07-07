from typing import List, Optional, Iterable, Callable, Union

from enum import Enum
import asyncio

import launch.logging
from launch.action import Action
from launch.actions import ExecuteProcess
from launch.events import matches_action
from launch.events.process import ProcessStarted, ProcessExited
from launch.events.process import ProcessStdout, ProcessStderr, ProcessStdin
from launch.events.process import ShutdownProcess
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


class ProcessStatus(Enum):
    STARTING = 'PROCESS_STATUS_STARTING'
    RUNNING = 'PROCESS_STATUS_RUNNING'
    SHUTTING_DOWN = 'PROCESS_STATUS_SHUTTING_DOWN'
    TERMINATED = 'PROCESS_STATUS_TERMINATED'


class Process(Action):

    def __init__(self, *, req_id, action: Union[ExecuteProcess, Node],
                 completed_future: asyncio.Future, 
                 events:List, plugin_id:str) -> None:

        if isinstance(action, ExecuteProcess) is False:
            raise RuntimeError("The action differs from Execute Process.")

        self.__action = action
        self.__completed_future = completed_future

        self.__launch_context = None
        self.__on_start_ext = None
        self.__on_exit_ext = None
        self.__on_stdout_ext = None
        self.__on_stderr_ext = None
        self.__on_stdin_ext = None

        self.__plugin_id = plugin_id
        self.__events = events

        self.__id = req_id
        self.__pid = None

        event_handlers = [
            OnProcessStart(target_action=action, on_start=self.__on_start),
            OnProcessExit(target_action=action, on_exit=self.__on_exit),
            OnProcessIO(target_action=action,
                        on_stdout=self.__on_stdout,
                        on_stderr=self.__on_stderr,
                        on_stdin=self.__on_stdin)
        ]
        self.__event_handlers = event_handlers
        self._status = ProcessStatus.STARTING
        self.__logger = launch.logging.get_logger('Process')

        super().__init__()

    def __on_start(self, event: ProcessStarted, context: LaunchContext):
        self.__pid = event.pid
        self.__completed_future.set_result(event)
        self.__launch_context = context
        
        self._status = ProcessStatus.RUNNING

        if self.__on_start_ext is not None:
            self.__on_start_ext(self, event)

    def __on_exit(self, event: ProcessExited, context: LaunchContext):
        for event_handler in self.__event_handlers:
            self.__launch_context.unregister_event_handler(event_handler)

        self._status = ProcessStatus.TERMINATED

        if self.__on_exit_ext is not None:
            self.__on_exit_ext(self, event)

    def __on_stdout(self, event: ProcessStdout):
        if self.__on_stdout_ext is not None:
            self.__on_stdout_ext(self, event)

    def __on_stderr(self, event: ProcessStderr):
        if self.__on_stderr_ext is not None:
            self.__on_stderr_ext(self, event)

    def __on_stdin(self, event: ProcessStdin):
        pass

    def shutdown(self):
        if self.__action is None:
            return
        if self.__launch_context is None:
            return

        self._status = ProcessStatus.SHUTTING_DOWN
        event = ShutdownProcess(process_matcher=matches_action(self.__action))
        self.__launch_context.emit_event_sync(event)

    def is_emit(self, event_type:str)->bool:
        return event_type in self.__events

    @property
    def action(self) -> Action:
        return self.__action

    @property
    def status(self) -> ProcessStatus:
        return self._status

    @property
    def id(self) -> int:
        return self.__id
    
    @property
    def pid(self) -> int:
        return self.__pid

    @property
    def plugin_id(self) -> str:
        return self.__plugin_id

    @property
    def on_start(self):
        return self.__on_start_ext

    @on_start.setter
    def on_start(self, func: Callable[['Process', ProcessStarted], None]):
        if callable(func):
            self.__on_start_ext = func

    @property
    def on_exit(self):
        return self.__on_exit_ext

    @on_exit.setter
    def on_exit(self, func: Callable[['Process', ProcessExited], None]):
        if callable(func):
            self.__on_exit_ext = func

    @property
    def on_stdout(self):
        return self.__on_stdout_ext

    @on_stdout.setter
    def on_stdout(self, func: Callable[['Process', ProcessStdout], None]):
        if callable(func):
            self.__on_stdout_ext = func

    @property
    def on_stderr(self):
        return self.__on_stderr_ext

    @on_stderr.setter
    def on_stderr(self, func: Callable[['Process', ProcessStderr], None]):
        if callable(func):
            self.__on_stderr_ext = func

    @property
    def event_handlers(self):
        return self.__event_handlers
