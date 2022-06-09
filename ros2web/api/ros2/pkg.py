import functools
from token import OP
from typing import Optional, Union
from typing import Any, Dict, List
import os
from dacite import from_dict
import launch.logging
import dataclasses
from asyncio import AbstractEventLoop

from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
from ament_index_python import get_package_prefix
from catkin_pkg.package import parse_package, Package as RosPackage
from ros2pkg.api import get_package_names
from ros2pkg.api import get_executable_paths

from ..models.package import PackageManifest, Package
from ...utilities.converter import convert_catkin_pkg_to_dict


def _get_package_manifest_by_path(package_share_dir) -> Optional[PackageManifest]:
    package_xml = os.path.join(package_share_dir, 'package.xml')
    if not os.path.isfile(package_xml):
        return None
    package: RosPackage = parse_package(package_xml)
    package_dict = convert_catkin_pkg_to_dict(package)
    return from_dict(data_class=PackageManifest, data=package_dict)    


def _get_executables(package_name: str) -> Optional[List[str]]:
    paths = get_executable_paths(package_name=package_name)
    if paths is not None:
        return sorted(set([os.path.basename(path) for path in paths]))
    else:
        return None

def _get_package_executables(package_name:str=None) -> List[List[str]]:
    package_executables = []
    if package_name:
        executables = _get_executables(package_name)
        package_executables = \
            [[package_name, executable] for executable in executables]
    else:
        package_names = _get_package_names()
        for package_name in package_names:
            executables = _get_executables(package_name)
            _package_executables = [[package_name, executable] for executable in executables]
            package_executables.extend(_package_executables)
            
    return package_executables

def _get_package_names() -> List[str]:
    return sorted(get_package_names())


def _get_package(package_name) -> Optional[Package]:
    
    try:
        package_share_dir = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return None
    
    manifest = _get_package_manifest_by_path(package_share_dir)
    executables = _get_executables(package_name)
    
    return Package(name=package_name, executables=executables, manifest=manifest)


class ROS2PackageAPI:
    def __init__(self, ros_node, *, loop: AbstractEventLoop) -> None:
        self.__ros_node = ros_node
        self.__loop = loop
        self.__logger = launch.logging.get_logger('ROS2PackageAPI')

    async def package(self, package_name) -> Optional[PackageManifest]:
        descriptor = await self.__loop.run_in_executor(None, _get_package, package_name)
        return descriptor

    async def list(self) -> List[str]:
        return await self.__loop.run_in_executor(None, _get_package_names)
    
    async def executables(self, package_name:str=None) -> List[str]:
        func = functools.partial(_get_package_executables, package_name=package_name)
        return await self.__loop.run_in_executor(None, func)