
import asyncio
import dataclasses

import launch.logging
from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.direct import DirectNode
from ros2web.api import ros2


def test_interface_list():
    logger = launch.logging.get_logger('test_interface_list')
    loop = asyncio.get_event_loop()
    with DirectNode([], node_name='test') as node:
        interface_api = ros2.ROS2InterfaceAPI(ros_node=node, loop=loop)
        interfaces = loop.run_until_complete(interface_api.list())
    
    for interface in interfaces:
        print(interface)
    
    loop.close()

