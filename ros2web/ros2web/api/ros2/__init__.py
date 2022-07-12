from typing import Optional, Iterable, Callable, Awaitable, TYPE_CHECKING, NamedTuple
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
# from launch.actions import ExecuteProcess
# from launch_ros.actions import Node

from .node import ROS2NodeAPI
from .param import ROS2ParamAPI
from .pkg import ROS2PackageAPI
from .interface import ROS2InterfaceAPI
from .topic import ROS2TopicAPI

from ...launch.actions.node_wrapper import NodeWrapper, ExecuteProcessWrapper

if TYPE_CHECKING:
    from ...launch.service import DynamicLaunchService, Process
    

class ProcessStatus(Enum):
    STARTING = 'PROCESS_STATUS_STARTING'
    RUNNING = 'PROCESS_STATUS_RUNNING'
    SHUTTING_DOWN = 'PROCESS_STATUS_SHUTTING_DOWN'
    TERMINATED = 'PROCESS_STATUS_TERMINATED'


class ProcessEvent(NamedTuple):
    event_type: str
    pid: int
    text: Optional[bytes]
    
class ROS2API:
    def __init__(self, *, ros_node: ROSNode, launch_service: 'DynamicLaunchService', loop: AbstractEventLoop = None) -> None:
        self.__ros_node = ros_node
        self.__ros2_pkg_api = ROS2PackageAPI(ros_node, loop=loop)
        self.__ros2_node_api = ROS2NodeAPI(ros_node, loop=loop)
        self.__ros2_param_api = ROS2ParamAPI(ros_node, loop=loop)
        self.__ros2_interface_api = ROS2InterfaceAPI(ros_node, loop=loop)
        self.__ros2_topic_api = ROS2TopicAPI(ros_node, loop=loop)
        self.__launch_service = launch_service

    async def run(self, executable: SomeSubstitutionsType,
                  package: Optional[SomeSubstitutionsType] = None,
                  name: Optional[SomeSubstitutionsType] = None,
                  namespace: Optional[SomeSubstitutionsType] = None,
                  exec_name: Optional[SomeSubstitutionsType] = None,
                  parameters: Optional[SomeParameters] = None,
                  remappings: Optional[SomeRemapRules] = None,
                  arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
                  ) -> Optional['Process']:

        action = NodeWrapper(executable=executable,
                      package=package,
                      name=name,
                      namespace=namespace,
                      exec_name=exec_name,
                      parameters=parameters,
                      remappings=remappings,
                      arguments=arguments)

        return self.__launch_service.execute_process(action=action)

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
