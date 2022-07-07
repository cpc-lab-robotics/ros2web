from typing import Dict

from ros2web.api import Plugin
from ros2web.api import RouteTableDef, Request

from std_msgs.msg import String

routes = RouteTableDef()


class ROS2WebExtension(Plugin):

    def __init__(self, config: Dict) -> None:
        super().__init__(
            routes=routes
        )
        self.__config = config
        self.__publisher = None

    async def on_startup(self):
        self.__publisher = self.ros_node.create_publisher(String, 'topic', 10)

    async def on_shutdown(self):
        self.ros_node.destroy_publisher(self.__publisher)
    
    @@routes.get('/click')
    async def click_request(self, request: Request):
        string = String()
        string.data = "Hello, world"
        self.__publisher.publish(string)
