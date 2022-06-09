from typing import Optional, Union, Iterable
from typing import List, Dict, Any

import functools
from asyncio import AbstractEventLoop
from dacite import from_dict

from rosidl_runtime_py import get_interface_packages
from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_service_interfaces
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py import utilities
import launch.logging
from ..models.package import Package
from ..models.interface import Msg, Srv, Act


class ROS2InterfaceAPI:
    def __init__(self, ros_node, *, loop: AbstractEventLoop) -> None:
        self.__ros_node = ros_node
        self.__loop = loop
        self.__logger = launch.logging.get_logger('ROS2InterfaceAPI')

    async def packages(self)->List[Package]:
        pass
    
    async def list(self, package_names: Iterable[str] = [])->List[Union[Msg, Srv, Act]]:
        interfaces = []
        
        message_interfaces = await self.__loop.run_in_executor(None, get_message_interfaces, package_names)
        for package_name in sorted(message_interfaces):
            for message_name in sorted(message_interfaces[package_name]):
                name = message_name.split('/')[1]
                msg = Msg(name=name, package_name=package_name)
                interfaces.append(msg)
                
        service_interfaces = await self.__loop.run_in_executor(None, get_service_interfaces, package_names)
        for package_name in sorted(service_interfaces):
            for service_name in sorted(service_interfaces[package_name]):
                name = service_name.split('/')[1]
                msg = Srv(name=name, package_name=package_name)
                interfaces.append(msg)
                
        action_interfaces = await self.__loop.run_in_executor(None, get_action_interfaces, package_names)
        for package_name in sorted(action_interfaces):
            for action_name in sorted(action_interfaces[package_name]):
                name = action_name.split('/')[1]
                msg = Act(name=name, package_name=package_name)
                interfaces.append(msg)
                
        return interfaces
    
    async def proto(self, interface_name: str):
        interface = utilities.get_interface(interface_name)
        if utilities.is_action(interface):
            instance = interface.Goal()
        elif utilities.is_service(interface):
            instance = interface.Request()
        else:
            instance = interface()
        return message_to_ordereddict(instance)
    
    async def show(self):
        pass
