from typing import TYPE_CHECKING

from launch.event import Event

if TYPE_CHECKING:
    from ...api import Plugin   # noqa: F401

class ExecutionPlugin(Event):
    name = 'ros2web.launch.events.ExecutionPlugin'
    
    def __init__(self, *, plugin: 'Plugin') -> None:
        self.__plugin = plugin
    
    @property
    def plugin(self):
        return self.__plugin
