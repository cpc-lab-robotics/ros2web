from typing import Optional, Tuple, Union, Iterable
from typing import List, Dict, Any, OrderedDict

import functools
from asyncio import AbstractEventLoop
from dacite import from_dict

from rosidl_runtime_py import get_interface_packages
from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_service_interfaces
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py import utilities
from ros2interface.verb.show import _get_interface_lines, InterfaceTextLine


import launch.logging
from ..models.package import PackageManifest
from ..models.interface import Msg, Srv, Act, Field
from ..models import dict_to_field

logger = launch.logging.get_logger('api.ros2.interface')


def _get_interface_field(line: InterfaceTextLine):

    text = str(line)
    if not text or line.is_comment():
        return None
    elif line.is_trailing_comment():
        comment_start_idx = text.find(line.trailing_comment)
        text = text[:comment_start_idx - 1].strip()

    if text.startswith('---'):
        return '---'

    return text.split(' ')


def _get_msg_interface(interface_identifier: str) -> List[Field]:
    fields = []
    for line in _get_interface_lines(interface_identifier):
        field_data = _get_interface_field(line)
        if isinstance(field_data, list) and len(field_data) > 1:
            field = Field(type=field_data[0], name=field_data[1])
            fields.append(field)
            if line.nested_type:
                nested = _get_msg_interface(line.nested_type)
                field.nested = nested
    return fields


def _get_srv_interface(interface_identifier: str):
    request = []
    response = []
    tmp = request
    for line in _get_interface_lines(interface_identifier):
        field_data = _get_interface_field(line)
        
        if field_data == '---':
            tmp = response
        elif isinstance(field_data, list) and len(field_data) > 1:
            field = Field(type=field_data[0], name=field_data[1])
            tmp.append(field)
            if line.nested_type:
                nested = _get_msg_interface(line.nested_type)
                field.nested = nested
    return request, response


def _get_interface(interface_identifier: str) -> Optional[Union[Msg, Srv, Act]]:
    try:
        [package_name, interface_type, name] = interface_identifier.split('/')

        if interface_type == 'msg':
            fields = _get_msg_interface(interface_identifier)
            return Msg(name=name, package_name=package_name, fields=fields)
        elif interface_type == 'srv':
            request, response = _get_srv_interface(interface_identifier)
            return Srv(name=name, package_name=package_name, request=request, response=response)
        elif interface_type == 'action':
            pass
    except Exception as e:
        logger.error(f"{interface_identifier}: {e}")
    return


class ROS2InterfaceAPI:
    def __init__(self, ros_node, *, loop: AbstractEventLoop) -> None:
        self.__ros_node = ros_node
        self.__loop = loop

    async def packages(self) -> List[PackageManifest]:
        pass

    #
    async def list(self, package_names: Iterable[str] = []) -> List[str]:
        interfaces = []

        message_interfaces = await self.__loop.run_in_executor(None, get_message_interfaces, package_names)
        interfaces = [f'{package_name}/{message_name}' for package_name in sorted(message_interfaces)
                      for message_name in sorted(message_interfaces[package_name])]

        service_interfaces = await self.__loop.run_in_executor(None, get_service_interfaces, package_names)
        interfaces.extend([f'{package_name}/{message_name}' for package_name in sorted(service_interfaces)
                           for message_name in sorted(service_interfaces[package_name])])

        action_interfaces = await self.__loop.run_in_executor(None, get_action_interfaces, package_names)
        interfaces.extend([f'{package_name}/{message_name}' for package_name in sorted(action_interfaces)
                           for message_name in sorted(action_interfaces[package_name])])

        return interfaces

    async def proto(self, interface_name: str) -> OrderedDict:
        interface = utilities.get_interface(interface_name)
        if utilities.is_action(interface):
            instance = interface.Goal()
        elif utilities.is_service(interface):
            instance = interface.Request()
        else:
            instance = interface()
        return message_to_ordereddict(instance)

    async def describe(self, interface_name: str) -> Optional[Union[Msg, Srv, Act]]:
        return await self.__loop.run_in_executor(None, _get_interface, interface_name)
