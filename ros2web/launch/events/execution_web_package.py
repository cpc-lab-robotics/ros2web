from typing import TYPE_CHECKING

from launch.event import Event

if TYPE_CHECKING:
    from ...api import WebPackage   # noqa: F401

class ExecutionWebPackage(Event):
    name = 'ros2web.launch.events.ExecutionWebPackage'
    
    def __init__(self, *, web_package: 'WebPackage') -> None:
        self.__web_package = web_package
    
    @property
    def web_package(self):
        return self.__web_package
