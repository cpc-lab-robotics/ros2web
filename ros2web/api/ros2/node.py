from typing import List, Dict, Any, Optional
import functools
from asyncio import AbstractEventLoop
from dacite import from_dict

import launch.logging
from ros2node.api import get_node_names, NodeName, TopicInfo
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info
from ros2node.api import has_duplicates

from ros2cli.node.strategy import NodeStrategy

from ..models.node import Node, NodeInterface
from ..models.topic import Topic
from ..models.service import Service
from ..models.action import Action
from ...utilities import UniqueName

logger = launch.logging.get_logger(__file__)


def _topic(topics: List[TopicInfo]) -> List[Topic]:
    return [Topic(name=topic.name, type=type)
            for topic in topics
            for type in topic.types
            if type.startswith('rcl_interfaces') is False]


def _service(topics: List[TopicInfo]) -> List[Service]:
    return [Service(name=topic.name, type=type)
            for topic in topics
            for type in topic.types
            if type.startswith('rcl_interfaces') is False]


def _action(topics: List[TopicInfo]) -> List[Action]:
    return [Action(name=topic.name, type=type)
            for topic in topics
            for type in topic.types
            if type.startswith('rcl_interfaces') is False]


def _get_node_interface(*, node_names: List[str], include_hidden: bool = False) -> List[NodeInterface]:

    interfaces = []
    with NodeStrategy({}) as node:
        for node_name in node_names:
            subscribers = get_subscriber_info(
                node=node, remote_node_name=node_name, include_hidden=include_hidden)
            publishers = get_publisher_info(
                node=node, remote_node_name=node_name, include_hidden=include_hidden)
            service_servers = get_service_server_info(
                node=node, remote_node_name=node_name, include_hidden=include_hidden)
            service_clients = get_service_client_info(
                node=node, remote_node_name=node_name, include_hidden=include_hidden)
            action_servers = get_action_server_info(
                node=node, remote_node_name=node_name, include_hidden=include_hidden)
            action_clients = get_action_client_info(
                node=node, remote_node_name=node_name, include_hidden=include_hidden)

            interface_data = {
                "node_name": node_name,
                "subscribers": _topic(subscribers),
                "publishers": _topic(publishers),
                "service_servers": _service(service_servers),
                "service_clients": _service(service_clients),
                "action_servers": _action(action_servers),
                "action_clients": _action(action_clients)
            }
            interfaces.append(NodeInterface(**interface_data))
    return interfaces


def _get_node_names(ros_node):
    node_names: List[NodeName] = get_node_names(node=ros_node)

    sorted_names = sorted(n.full_name for n in node_names)
    if has_duplicates(sorted_names):
        logger.warning('Be aware that are nodes in the graph that share an exact name, '
                       'this can have unintended side effects.')

    unique_name = UniqueName(prefix="node")
    return [Node(id=unique_name.get_name(node_name.full_name),
                 name=node_name.name,
                 namespace=node_name.namespace,
                 full_name=node_name.full_name)
            for node_name in node_names]


class ROS2NodeAPI:
    def __init__(self, ros_node, *, loop: AbstractEventLoop) -> None:
        self.__ros_node = ros_node
        self.__loop = loop

    async def info(self, node_names: List[str]) -> List[NodeInterface]:
        func = functools.partial(_get_node_interface, node_names=node_names)
        interface = await self.__loop.run_in_executor(None, func)
        return interface

    async def list(self) -> List[Node]:
        nodes = await self.__loop.run_in_executor(None, _get_node_names, self.__ros_node)
        return nodes
