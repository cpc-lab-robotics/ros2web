from typing import Dict

from ros2web.api import Plugin
from ros2web.api import RouteTableDef, Request
from ros2web.api import WidgetEvent

from std_msgs.msg import String

routes = RouteTableDef()


class ROS2WebPackage(Plugin):

    def __init__(self, config: Dict) -> None:
        init_state = {
            'button_label': 'Hello, world'
        }

        super().__init__(
            init_state=init_state,
            routes=routes,
        )
        self.__config = config
        self.__publisher = None

    @@routes.page
    async def page(self, request: Request):
        return self.__config['page']

    async def on_startup(self):
        self.bind("button", "on_click", self.click_handler)
        self.__publisher = self.ros_node.create_publisher(String, 'topic', 10)

    async def on_shutdown(self):
        self.ros_node.destroy_publisher(self.__publisher)
    
    async def click_handler(self, event: WidgetEvent):
        string = String()
        string.data = "Hello, world"
        self.__publisher.publish(string)
        
    @@routes.get("/package_names")
    async def package_names(self, request: Request):
        return await self.ros2.pkg.list()
