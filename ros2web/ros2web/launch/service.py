from typing import Optional, Union, Iterable, Callable, NamedTuple, Awaitable, TYPE_CHECKING
from typing import List, Dict, Set

from enum import Enum
import asyncio
from asyncio import AbstractEventLoop
import functools
import uuid
import time
import multiprocessing as mp
from multiprocessing import JoinableQueue
from multiprocessing.connection import Connection
import concurrent.futures
import concurrent.futures.process
import threading

import launch.logging
from launch import LaunchService, LaunchDescription

from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from .actions.process_manager import ProcessManager


from ..utilities import logging_screen_handler_remover, logging_logfile_handler_remover

from ..api.ros2 import ProcessEvent, ProcessStatus

from .actions.node_wrapper import NodeWrapper, ExecuteProcessWrapper


class Process:
    def __init__(self, connection: Connection) -> None:
        self.__connection = connection
        self.__event_handler_dict: Dict[str, Callable[[
            ProcessEvent], Awaitable[None]]] = {}
        self.__loop = asyncio.get_event_loop()

        self.__pid = None

        self.__status = ProcessStatus.STARTING

        self.__is_running = True
        self.__loop.run_in_executor(None, self.__update_status)

        self.__logger = launch.logging.get_logger('launch.service.process')

    def __event_handler(self, event: ProcessEvent):
        if event.event_type == "start":
            self.__status = ProcessStatus.RUNNING
        elif event.event_type == "exit":
            self.__status = ProcessStatus.TERMINATED
            self.__is_running = False
            self.__connection.close()

        handler = self.__event_handler_dict.get(event.event_type)
        if handler is not None:
            try:
                if self.__loop.is_closed() is False:
                    asyncio.run_coroutine_threadsafe(
                        handler(event), self.__loop)
            except Exception as e:
                self.__logger.error(e)

    def __update_status(self):
        while self.__is_running:
            try:
                data = self.__connection.recv()
                if isinstance(data, ProcessEvent):
                    self.__event_handler(data)
                else:
                    pass
            except ValueError as e:
                pass
            except EOFError:
                pass

    def shutdown(self):
        if self.__status == ProcessStatus.RUNNING:
            self.__status = ProcessStatus.SHUTTING_DOWN
            self.__connection.send({
                'method_name': 'shutdown',
                'args': [],
                'kwargs': {},
            })
    
    @property
    def pid(self) -> Optional[int]:
        return self.__pid

    @property
    def status(self) -> ProcessStatus:
        return self.__status

    @property
    def on_start(self):
        return self.__event_handler_dict.get('start')

    @on_start.setter
    def on_start(self, func: Callable[[ProcessEvent], Awaitable[None]]):
        if callable(func):
            self.__event_handler_dict['start'] = func

    @property
    def on_exit(self):
        return self.__event_handler_dict.get('exit')

    @on_exit.setter
    def on_exit(self, func: Callable[[ProcessEvent], Awaitable[None]]):
        if callable(func):
            self.__event_handler_dict['exit'] = func

    @property
    def on_stdout(self):
        return self.__event_handler_dict.get('stdout')

    @on_stdout.setter
    def on_stdout(self, func: Callable[[ProcessEvent], Awaitable[None]]):
        if callable(func):
            self.__event_handler_dict['stdout'] = func

    @property
    def on_stderr(self):
        return self.__event_handler_dict.get('stderr')

    @on_stderr.setter
    def on_stderr(self, func: Callable[[ProcessEvent], Awaitable[None]]):
        if callable(func):
            self.__event_handler_dict['stderr'] = func


class DynamicLaunchService(mp.Process):
    def __init__(self) -> None:
        super().__init__()
        self.__logger = launch.logging.get_logger('DynamicLaunchService')

        self.__processes: Set[Process] = set()
        self.__call_lock = threading.Lock()

        parent_conn, child_conn = mp.Pipe()
        self.__parent_conn = parent_conn
        self.__child_conn = child_conn

        self.__is_running = mp.Value('i', False)

    @property
    def is_running(self) -> bool:
        return bool(self.__is_running.value)

    def execute_process(self, *, action: Union[Node, ExecuteProcess]) -> Optional[Process]:
        if isinstance(action, ExecuteProcess) is False:
            raise RuntimeError("The action differs from Execute Process.")

        kwargs = {
            'action': action
        }

        response = self.__call_method("_execute_process", [], kwargs)
        if isinstance(response, Connection) is False:
            return None

        process = Process(response)
        self.__processes.add(process)

        return process

    async def _execute_process(self, *, action: Union[Node, ExecuteProcess]):
        parent_conn, child_conn = mp.Pipe()

        await self.__process_manager.execute_process(action=action, connection=parent_conn)

        return child_conn

    def shutdown(self):
        time.sleep(0.1)
        if self.is_running == True:
            self.__call_method("_shutdown", [], {})

    async def _shutdown(self):
        future = self.__launch_service.shutdown()
        if future is not None:
            await future

    def run(self):
        try:
            self.__is_running.value = True
            self.__process_manager = ProcessManager()
            self.__loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.__loop)

            self.__launch_service = LaunchService(argv=[], debug=False)
            self.__loop.run_in_executor(None, self.__method_loop)

            self.__launch_service.context.extend_globals({})
            logging_screen_handler_remover(
                self.__launch_service._LaunchService__logger)
            logging_logfile_handler_remover(
                self.__launch_service._LaunchService__logger)
            launch_description = LaunchDescription([
                self.__process_manager
            ])
            self.__launch_service.include_launch_description(
                launch_description)
            self.__launch_service.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.__is_running.value = False
            self.__loop.close()

    def __call_method(self, method_name, args, kwargs):
        if self.is_running == False:
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
                if self.__parent_conn.poll(timeout=1.5):
                    response = self.__parent_conn.recv()
        except EOFError as e:
            self.__logger.error("[call_api] EOFError: {}".format(e))
        except ValueError as e:
            self.__logger.error("[call_api] ValueError: {}".format(e))
        except BrokenPipeError as e:
            self.__logger.error("[call_api] BrokenPipeError: {}".format(e))

        return response

    def __call(self, data, timeout: float = 1.0):
        try:
            method_name: str = data.get('method_name')
            args = data.get('args', [])
            kwargs = data.get('kwargs', {})
            response = None

            if self.__loop.is_closed() == False:
                future = asyncio.run_coroutine_threadsafe(
                    getattr(self, method_name)(*args, **kwargs),
                    self.__loop)
                response = future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            self.__logger.error(
                'The coroutine took too long, cancelling the task...')
            future.cancel()
        except Exception as e:
            self.__logger.error('call_service[{}]: {}'.format(method_name, e))
        return response

    def __method_loop(self):
        while self.__is_running:
            try:
                self.__child_conn.send(
                    self.__call(self.__child_conn.recv())
                )
            except ValueError as e:
                self.__logger.error('service_loop: '.format(e))
            except EOFError:
                pass
            except Exception as e:
                self.__logger.error('service_loop: '.format(e))
