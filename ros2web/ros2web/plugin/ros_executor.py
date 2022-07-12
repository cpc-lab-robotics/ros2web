from typing import Optional, List
from asyncio import AbstractEventLoop
import concurrent.futures

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
import launch.logging


class ROSExecutor:
    def __init__(self, *,
                 node_name: str,
                 namespace: Optional[str] = None,
                 args: Optional[List[str]] = None,
                 loop: AbstractEventLoop) -> None:

        self.__logger = launch.logging.get_logger('ROSExecutor')

        self.__loop = loop
        self.__ros_context = rclpy.Context()
        rclpy.init(args=args, context=self.__ros_context)
        self.__ros_executor = SingleThreadedExecutor(
            context=self.__ros_context)

        self.__node = rclpy.create_node(node_name,
                                        namespace=namespace,
                                        context=self.__ros_context
                                        )
        self.__ros_executor.add_node(self.__node)

        self.__is_running = True
        executor = concurrent.futures.ThreadPoolExecutor()
        self.__loop.run_in_executor(executor, self.__ros_loop)

    def __ros_loop(self):
        try:
            while self.__is_running:
                self.__ros_executor.spin_once(timeout_sec=1.0)
        except ExternalShutdownException:
            pass
            
    def shutdown(self):
        self.__is_running = False
        self.__ros_executor.remove_node(self.__node)
        self.__node.destroy_node()
        rclpy.shutdown(context=self.__ros_context)
        
        

    @property
    def node(self) -> Node:
        return self.__node
