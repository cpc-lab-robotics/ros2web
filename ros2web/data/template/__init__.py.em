import importlib.resources
import launch.logging

from ros2web.api import WebPackage
from ros2web.api import APIDefinition


with importlib.resources.path("@package_name", "data") as path:
    UI_FILE_PATH = path.joinpath("ui.xml")

api_def = APIDefinition()

class @inheritance_name:

    def __init__(self) -> None:
        init_state = {}

        super().__init__(
            init_state=init_state,
            xml=UI_FILE_PATH,
            api_def=api_def,
            )
        self.__logger = launch.logging.get_logger('@class_name')
    
    @@api_def.get("/package_names")
    async def package_names(self):
        return await self.ros2.pkg.list()

    @@api_def.get("/executables")
    async def executables(self):
        return await self.ros2.pkg.executables()

    @@api_def.get("/package/{package_name}")
    async def package(self, package_name):
        if package_name:
            package = await self.ros2.pkg.package(package_name)
        return package

    async def on_startup(self):
        pass

    async def on_shutdown(self):
        pass