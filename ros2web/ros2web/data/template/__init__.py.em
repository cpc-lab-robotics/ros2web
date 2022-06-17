import importlib.resources
import yaml

from ros2web.api import WebPackage
from ros2web.api import RouteTableDef, Request
from ros2web.api import WidgetEvent

import launch.logging
from std_msgs.msg import String

with importlib.resources.path("@package_name", "data") as path:
    CONFIG_FILE_PATH = path.joinpath("config.yml")

routes = RouteTableDef()


class @inheritance_name:

    def __init__(self) -> None:
        init_state = {
            'button_label': 'Hello'
        }
        
        super().__init__(
            init_state=init_state,
            routes=routes,
        )
        self.__config = None
        self.__logger = launch.logging.get_logger('@class_name')
        
        try:
            with open(CONFIG_FILE_PATH, 'r') as yml:
                self.__config = yaml.safe_load(yml)
        except yaml.YAMLError as e:
            self.__logger.error(f'(YAML) :{e}')
        
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
