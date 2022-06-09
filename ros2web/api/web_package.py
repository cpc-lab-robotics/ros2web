from ctypes import Union
from typing import Dict, List, Optional
from typing import TYPE_CHECKING
import asyncio

from rclpy.node import Node
from .ros2 import ROS2API
from .api_def import APIDefinition

if TYPE_CHECKING:
    from ..wp.wp_api import WebPackageAPI  # noqa: F401


class WebPackage:

    def __init__(self, *, init_state: Dict, 
                 xml: Optional[str] = None,
                 api_def: Optional[APIDefinition] = None,
                 ) -> None:
        self.__init_state = init_state
        self.__api: 'WebPackageAPI' = None
        self.__xml = xml
        self.__api_def = api_def

    def _set_api(self, api: 'WebPackageAPI'):
        api.init(web_package=self, state=self.__init_state,
                 xml=self.__xml,
                 api_def=self.__api_def)
        self.__api = api

    def bind(self, widget_id, event_type, handler):
        self.__api.bind(widget_id, event_type, handler)

    def unbind(self, widget_id, event_type):
        self.__api.unbind(self, widget_id, event_type)

    def set_state(self, state: Dict):
        self.__api.set_state(state)

    @property
    def state(self) -> Dict:
        return self.__api.state

    @property
    def web_package_name(self) -> str:
        return self.__api.name

    @property
    def ros2(self) -> ROS2API:
        return self.__api.ros2

    @property
    def ros_node(self) -> Node:
        return self.__api.ros2.ros_node

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        return self.__api.loop

    async def on_startup(self):
        raise NotImplementedError()

    async def on_shutdown(self):
        raise NotImplementedError()
