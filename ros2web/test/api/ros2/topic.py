
import asyncio
import dataclasses

import launch.logging
from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.direct import DirectNode
from ros2web.api import ros2


# def test_topic_list():
#     logger = launch.logging.get_logger('test_topic_list')
#     loop = asyncio.get_event_loop()
#     with DirectNode([], node_name='test') as node:
#         topic_api = ros2.ROS2TopicAPI(ros_node=node, loop=loop)
#         topics = loop.run_until_complete(topic_api.list())
    
#     for topic in topics:
#         print(topic)
#         # logger.info(dataclasses.asdict(topic))
#         # break
    
#     loop.close()

