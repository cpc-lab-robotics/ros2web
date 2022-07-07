from typing import Dict, cast

from ros2web.api import Plugin
from ros2web.api import RouteTableDef, Request
from ros2web.api.models import model_to_dict, dict_to_model
from ros2web.api.models import Param
from ros2web.api.ros2.param import RCLPY_PARAM_TYPE

from rclpy.parameter import Parameter
import launch.logging

routes = RouteTableDef()


class ROS2WebExtension(Plugin):
    
    def __init__(self, config: Dict) -> None:
        super().__init__(
            routes=routes
        )
        
        self.__config = config
        self.__logger = launch.logging.get_logger('ROS2WebExtension')

    async def on_startup(self):
        pass
    
    async def on_shutdown(self):
        pass
    
    @routes.post('/set')
    async def set_param(self, request: Request):
        param: Param = cast(Param, dict_to_model(request.json_payload))
        param_type = RCLPY_PARAM_TYPE.get(param.descriptor.paramType)
        parameter = Parameter(param.name, param_type, param.value)
        parameters = [parameter]
        
        results = await self.ros2.param.set(param.node_name, parameters)
        
        if results is not None and len(results) > 0:
            result = results[0]
            if result.successful:
                return request.json_payload
        return None
    
    @routes.post('/get')
    async def get_param(self, request: Request):
        node_name = request.json_payload.get('node_name')
        param_name = request.json_payload.get('param_name')
        params = await self.ros2.param.get(node_name, [param_name])
        
        if params is not None and len(params) > 0:
            return model_to_dict(params[0])
        else:
            return None
        
