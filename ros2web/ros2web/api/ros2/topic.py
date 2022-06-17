import threading
from typing import Optional, Union, Iterable
from typing import List, Dict, Any

import functools
from asyncio import AbstractEventLoop
from dacite import from_dict
import time

from rclpy.node import Node as ROSNode
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from ros2topic.api import get_topic_names_and_types
from ros2topic.api import get_msg_class
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py import utilities


import launch.logging

from ..models.topic import Topic
from ..models.interface import Msg


def _get_topic_names_and_types(*, ros_node, include_hidden_topics=False)-> List[Topic]:
    topic_names_and_types = get_topic_names_and_types(
        node=ros_node,
        include_hidden_topics=include_hidden_topics)
    
    def create_msg(type):
        t = type.split('/')
        return Msg(name=t[2], package_name=t[0])
    
    topics = [Topic(id=f'topic:{topic[0]}:{type}', name=topic[0], type=create_msg(type))
                for topic in topic_names_and_types
                for type in topic[1]]
    return topics



def _get_message(*, ros_node:ROSNode , topic:Topic)-> List[Topic]:
    event = threading.Event()
    instance = None
    def callback(msg):
        nonlocal event
        nonlocal message
        instance = msg
        event.set()
    
    message_type = get_message(topic.type.identifier)
    # message_type = get_msg_class(
            # ros_node, topic.name, include_hidden_topics=True)
    if message_type is None:
        raise RuntimeError()
    
    subscription = ros_node.create_subscription(message_type, topic.name, callback, 10)
    event.wait(timeout=0.5)
    
    if instance is None:
        interface = utilities.get_interface(topic.type.identifier)
        instance = interface()
    message = message_to_ordereddict(instance)
    ros_node.destroy_subscription(subscription)
    return message

class ROS2TopicAPI:
    def __init__(self, ros_node, *, loop: AbstractEventLoop) -> None:
        self.__ros_node = ros_node
        self.__loop = loop
        self.__logger = launch.logging.get_logger('ROS2TopicAPI')

    async def bw(self):
        pass

    async def delay(self):
        pass

    async def echo(self, topic:Topic):
        func = functools.partial(_get_message,
                         ros_node=self.__ros_node,
                         topic=topic)
        return await self.__loop.run_in_executor(None, func)

    async def find(self):
        pass

    async def hz(self):
        pass

    async def info(self):
        pass

    async def list(self, include_hidden_topics=True) -> List[Topic]:
        func = functools.partial(_get_topic_names_and_types,
                         ros_node=self.__ros_node,
                         include_hidden_topics=include_hidden_topics)

        return await self.__loop.run_in_executor(None, func)

    async def pub(self):
        pass

    async def type(self):
        pass
