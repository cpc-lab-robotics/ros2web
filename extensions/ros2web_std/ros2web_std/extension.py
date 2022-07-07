from typing import Dict

from ros2web.api import Plugin
from ros2web.api import RouteTableDef

routes = RouteTableDef()


class ROS2WebExtension(Plugin):
    
    def __init__(self, config: Dict) -> None:
        super().__init__(
            routes=routes
        )

    async def on_startup(self):
        pass

    async def on_shutdown(self):
        pass
