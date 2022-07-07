
import asyncio

import launch.logging
from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.direct import DirectNode
from ros2web.api import ros2


# def test_node_list():
#     logger = launch.logging.get_logger('test_node_list')
#     loop = asyncio.get_event_loop()
#     with DirectNode([], node_name='test') as node:
#         node_api = ros2.ROS2NodeAPI(ros_node=node, loop=loop)
#         result = loop.run_until_complete(node_api.list())
#         logger.info(result)

# def test_node_info():
#     logger = launch.logging.get_logger('test_node_info')
#     loop = asyncio.get_event_loop()
    
#     with DirectNode([], node_name='test') as node:
#         node_api = ros2.ROS2NodeAPI(ros_node=node, loop=loop)
#         node_name = 'turtlesim'
#         result = loop.run_until_complete(node_api.info(node_name))
#         logger.info(result)