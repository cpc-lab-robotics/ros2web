from typing import Dict, List, Any
from typing import TYPE_CHECKING, Optional, NamedTuple
import asyncio

from rclpy.node import Node
from .ros2 import ROS2API
from .route_table_def import RouteTableDef

if TYPE_CHECKING:
    from ..plugin.api import PluginAPI  # noqa: F401


class Plugin:

    def __init__(self, *, init_state: Dict={},
                 routes: Optional[RouteTableDef] = None
                 ) -> None:
        self.__init_state = init_state
        self.__api: 'PluginAPI' = None
        self.__ros2api: ROS2API = None
        self.__routes = routes
        
    def _set_api(self, api: 'PluginAPI'):
        api.init(plugin=self,
                 state=self.__init_state,
                 routes=self.__routes)
        self.__api = api
        
    def _set_ros2api(self, api: ROS2API):
        self.__ros2api = api
    
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
    def ros2(self) -> ROS2API:
        return self.__ros2api

    @property
    def ros_node(self) -> Node:
        return self.__ros2api.ros_node
    
    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        return self.__api.loop
    
    async def on_startup(self):
        raise NotImplementedError()

    async def on_shutdown(self):
        raise NotImplementedError()
