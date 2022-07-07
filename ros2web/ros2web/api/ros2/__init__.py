from typing import Optional, Union, Iterable, Callable, NamedTuple, Awaitable
from typing import List, Dict, Set
from enum import Enum
import asyncio
from asyncio import AbstractEventLoop
import functools
import uuid

from rclpy.node import Node as ROSNode

import launch.logging
from launch.some_substitutions_type import SomeSubstitutionsType
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules

from .node import ROS2NodeAPI
from .param import ROS2ParamAPI
from .pkg import ROS2PackageAPI
from .interface import ROS2InterfaceAPI
from .topic import ROS2TopicAPI


class ProcessStatus(Enum):
    PENDING = 'PROCESS_STATUS_PENDING'
    STARTING = 'PROCESS_STATUS_STARTING'
    RUNNING = 'PROCESS_STATUS_RUNNING'
    SHUTTING_DOWN = 'PROCESS_STATUS_SHUTTING_DOWN'
    TERMINATED = 'PROCESS_STATUS_TERMINATED'


class ProcessEvent(NamedTuple):
    pid: int
    type: str
    text: Optional[bytes]


class Process:

    def __init__(self, *, req_id,
                 on_start: Optional[Callable[[
                     ProcessEvent], Awaitable[None]]] = None,
                 on_exit: Optional[Callable[[ProcessEvent],
                                            Awaitable[None]]] = None,
                 on_stdout: Optional[Callable[[
                     ProcessEvent], Awaitable[None]]] = None,
                 on_stderr: Optional[Callable[[
                     ProcessEvent], Awaitable[None]]] = None,
                 cmd,
                 sys_service, loop: AbstractEventLoop) -> None:
        self.__id = req_id
        self.__cmd = cmd
        self.__pid = None
        self.__sys_service = sys_service
        self.__loop = loop

        self.__event_handler_dict: Dict[str, Callable[[ProcessEvent],
                                                      Awaitable[None]]] = {}

        if on_start is not None:
            self.__event_handler_dict['on_start'] = on_start
        if on_exit is not None:
            self.__event_handler_dict['on_exit'] = on_exit
        if on_stdout is not None:
            self.__event_handler_dict['on_stdout'] = on_stdout
        if on_stderr is not None:
            self.__event_handler_dict['on_stderr'] = on_stderr

        self.__status = ProcessStatus.PENDING

        self.__logger = launch.logging.get_logger(
            '[{} {}]:'.format(self.__cmd['package'],
                              self.__cmd['executable']))

    def _update_status(self, event: ProcessEvent):

        if event.type == "on_start":
            self.__pid = event.pid
            self.__status = ProcessStatus.RUNNING
        elif event.type == "on_exit":
            self.__status = ProcessStatus.TERMINATED

        handler = self.__event_handler_dict.get(event.type)
        if handler is not None:
            try:
                asyncio.run_coroutine_threadsafe(handler(event), self.__loop)
            except Exception as e:
                self.__logger.error(e)

    def shutdown(self):
        self.__status = ProcessStatus.SHUTTING_DOWN

        request = {
            'method_name': 'shutdown_process',
            'kwargs': {'pid': self.__pid}
        }
        call_sys_service = functools.partial(self.__sys_service, request)
        future = self.__loop.run_in_executor(None, call_sys_service)
        # def callback(future):
        #     print(future.result())
        # future.add_done_callback(callback)

    @property
    def id(self) -> Optional[int]:
        return self.__id

    @property
    def pid(self) -> Optional[int]:
        return self.__pid

    @property
    def status(self) -> ProcessStatus:
        return self.__status


class ROS2API:
    def __init__(self, *, ros_node: ROSNode, sys_service, loop: AbstractEventLoop = None) -> None:
        self.__loop = loop

        self.__processes: Set[Process] = set()
        
        self.__ros_node = ros_node
        self.__ros2_pkg_api = ROS2PackageAPI(ros_node, loop=loop)
        self.__ros2_node_api = ROS2NodeAPI(ros_node, loop=loop)
        self.__ros2_param_api = ROS2ParamAPI(ros_node, loop=loop)
        self.__ros2_interface_api = ROS2InterfaceAPI(ros_node, loop=loop)
        self.__ros2_topic_api = ROS2TopicAPI(ros_node, loop=loop)
        self.__sys_service = sys_service
        
        self.__logger = launch.logging.get_logger('ROS2API')
        

    def _emit_event(self, event):
        req_id = event.get("id")
        pid = event.get("pid")
        event_type = event.get("event_type")
        text = event.get("text")

        processes = [p for p in self.__processes if p.id == req_id]
        process = processes[0] if len(processes) > 0 else None

        if process is not None:
            process_event = ProcessEvent(pid=pid, type=event_type, text=text)
            process._update_status(process_event)
            if event_type == "on_exit":
                self.__processes.remove(process)

    async def run(self, executable: SomeSubstitutionsType,
                  package: Optional[SomeSubstitutionsType] = None,
                  name: Optional[SomeSubstitutionsType] = None,
                  namespace: Optional[SomeSubstitutionsType] = None,
                  exec_name: Optional[SomeSubstitutionsType] = None,
                  parameters: Optional[SomeParameters] = None,
                  remappings: Optional[SomeRemapRules] = None,
                  arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
                  on_start: Optional[Callable[[
                      ProcessEvent], Awaitable[None]]] = None,
                  on_exit: Optional[Callable[[ProcessEvent],
                                             Awaitable[None]]] = None,
                  on_stdout: Optional[Callable[[
                      ProcessEvent], Awaitable[None]]] = None,
                  on_stderr: Optional[Callable[[
                      ProcessEvent], Awaitable[None]]] = None,
                  ) -> Process:

        req_id = uuid.uuid4()

        events = []
        if on_stdout is not None:
            events.append('on_stdout')
        if on_stderr is not None:
            events.append('on_stderr')

        cmd = {
            'executable': executable,
            'package': package,
            'name': name,
            'namespace': namespace,
            'exec_name': exec_name,
            'parameters': parameters,
            'remappings': remappings,
            'arguments': arguments,
        }

        kwargs = {
            **cmd, 'req_id': req_id,  'events': events,
        }
        request = {
            'method_name': 'ros2_run',
            'kwargs': kwargs
        }

        process = Process(
            req_id=req_id,
            on_start=on_start,
            on_exit=on_exit,
            on_stdout=on_stdout,
            on_stderr=on_stderr,
            cmd=cmd,
            sys_service=self.__sys_service,
            loop=self.__loop)

        self.__processes.add(process)
        func = functools.partial(self.__sys_service, request)
        response = await self.__loop.run_in_executor(None, func)
        return process if req_id == response else None

    @property
    def pkg(self) -> ROS2PackageAPI:
        return self.__ros2_pkg_api

    @property
    def node(self) -> ROS2NodeAPI:
        return self.__ros2_node_api

    @property
    def param(self) -> ROS2ParamAPI:
        return self.__ros2_param_api

    @property
    def interface(self) -> ROS2InterfaceAPI:
        return self.__ros2_interface_api
    
    @property
    def topic(self) -> ROS2TopicAPI:
        return self.__ros2_topic_api
    
    @property
    def ros_node(self) -> ROSNode:
        return self.__ros_node