import imp
from typing import Optional, cast, Union
from typing import List, Dict
from dacite import from_dict
import dataclasses
from asyncio import AbstractEventLoop

import rclpy
import rclpy.parameter
from rclpy.parameter import Parameter as RCLPY_Parameter
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ListParametersResult
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import ListParameters
import launch.logging

from ..models.param import Param, ParamDescriptor, ParamType
from ..models import custom_asdict_factory

PARAM_TYPE = {
    ParameterType.PARAMETER_BOOL: ParamType.BOOL,
    ParameterType.PARAMETER_INTEGER: ParamType.INTEGER,
    ParameterType.PARAMETER_DOUBLE: ParamType.DOUBLE,
    ParameterType.PARAMETER_STRING: ParamType.STRING,
    ParameterType.PARAMETER_BYTE_ARRAY: ParamType.BYTE_ARRAY,
    ParameterType.PARAMETER_BOOL_ARRAY: ParamType.BOOL_ARRAY,
    ParameterType.PARAMETER_INTEGER_ARRAY: ParamType.INTEGER_ARRAY,
    ParameterType.PARAMETER_DOUBLE_ARRAY: ParamType.DOUBLE_ARRAY,
    ParameterType.PARAMETER_STRING_ARRAY: ParamType.STRING_ARRAY,
    ParameterType.PARAMETER_NOT_SET: ParamType.NOT_SET,
}

RCLPY_PARAM_TYPE = {
    ParamType.BOOL: RCLPY_Parameter.Type.BOOL,
    ParamType.INTEGER: RCLPY_Parameter.Type.INTEGER,
    ParamType.DOUBLE: RCLPY_Parameter.Type.DOUBLE,
    ParamType.STRING: RCLPY_Parameter.Type.STRING,
    ParamType.BYTE_ARRAY: RCLPY_Parameter.Type.BYTE_ARRAY,
    ParamType.BOOL_ARRAY: RCLPY_Parameter.Type.BOOL_ARRAY,
    ParamType.INTEGER_ARRAY: RCLPY_Parameter.Type.INTEGER_ARRAY,
    ParamType.DOUBLE_ARRAY: RCLPY_Parameter.Type.DOUBLE_ARRAY,
    ParamType.STRING_ARRAY: RCLPY_Parameter.Type.STRING_ARRAY,
    ParamType.NOT_SET: RCLPY_Parameter.Type.NOT_SET,
}

PARAM_PROP = {
    ParameterType.PARAMETER_BOOL: 'bool_value',
    ParameterType.PARAMETER_INTEGER: 'integer_value',
    ParameterType.PARAMETER_DOUBLE: 'double_value',
    ParameterType.PARAMETER_STRING: 'string_value',
    ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array_value',
    ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
    ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
    ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
    ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value',
    ParameterType.PARAMETER_NOT_SET: 'not_set',
}


class ROSParamError(RuntimeError):
    pass


def _get_param_type(parameter_type: int) -> ParamType:
    param_type = PARAM_TYPE.get(parameter_type, None)
    if param_type is None:
        raise ROSParamError(f"Unknown parameter type ({parameter_type})")
    return param_type


def _get_param_value(param: ParameterValue):
    value = None
    prop = PARAM_PROP.get(param.type, None)
    if prop is not None:
        value = getattr(param, prop, None)
    else:
        raise ROSParamError(f"Unknown parameter type '{param.type}'")
    return value


def _get_integer_range(values):
    if len(values) != 1:
        return None
    range: IntegerRange = values[0]
    return {
        'from_value': range.from_value,
        'to_value': range.to_value,
        'step': range.step
    }


def _get_floating_point_range(values):
    if len(values) != 1:
        return None
    range: FloatingPointRange = values[0]
    return {
        'from_value': range.from_value,
        'to_value': range.to_value,
        'step': range.step
    }


def _convert_descriptor(descriptor: ParameterDescriptor) -> ParamDescriptor:
    
    descriptor_data = {
        'name': descriptor.name,
        'type': _get_param_type(descriptor.type).value,
        'description': descriptor.description,
        'additional_constraints': descriptor.additional_constraints,
        'read_only': descriptor.read_only,
        'floating_point_range': _get_floating_point_range(descriptor.floating_point_range),
        'integer_range': _get_integer_range(descriptor.integer_range)
    }
    return from_dict(data_class=ParamDescriptor, data=descriptor_data)


async def _set_param(*, node_name,
                     parameters: List[rclpy.parameter.Parameter],
                     ros_node: Node) -> List[SetParametersResult]:

    parameter_msgs: List[Parameter] = []
    for parameter in parameters:
        parameter_msgs.append(parameter.to_parameter_msg())
    request = SetParameters.Request()
    request.parameters = parameter_msgs

    client = ros_node.create_client(
        SetParameters, f'{node_name}/set_parameters')

    ready = client.wait_for_service(timeout_sec=1.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    response = await client.call_async(request)

    ros_node.destroy_client(client)
    
    return response.results


async def _get_param(*, node_name: str,
                     parameter_names: List[str],
                     describe: bool,
                     ros_node: Node) -> Optional[List[Param]]:

    client = ros_node.create_client(
        GetParameters, f'{node_name}/get_parameters')
    ready = client.wait_for_service(timeout_sec=1.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')
    request = GetParameters.Request()
    request.names = parameter_names
    response = await client.call_async(request)
    ros_node.destroy_client(client)

    if response is None or type(response.values) is not list:
        raise RuntimeError('Response value does not exist.')
    if len(response.values) != len(parameter_names):
        raise RuntimeError('Failed to get parameters.')

    parameters = []
    
    if describe:
        client2 = ros_node.create_client(
            DescribeParameters, f'{node_name}/describe_parameters')

        ready2 = client2.wait_for_service(timeout_sec=1.0)
        if not ready2:
            raise RuntimeError('Wait for service timed out')

        request2 = DescribeParameters.Request()
        request2.names = parameter_names
        response2 = await client2.call_async(request2)
        ros_node.destroy_client(client2)

        if len(response.values) != len(response2.descriptors):
            raise RuntimeError('Failed to get parameters.')

        for descriptor, value in zip(response2.descriptors, response.values):
            param_descriptor = _convert_descriptor(descriptor)
            param_value = _get_param_value(value)
            
            
            param = Param(node_name=node_name, name=param_descriptor.name,
                          value=param_value, descriptor=param_descriptor)
            parameters.append(param)
    else:
        for param_name, value in zip(parameter_names, response.values):
            param_value = _get_param_value(value)
            param = Param(node_name=node_name, name=param_name, value=param_value)
            parameters.append(param)
    return parameters


async def _get_param_descriptor(*, node_name: str,
                                parameter_names: List[str],
                                ros_node: Node) -> Optional[List[ParamDescriptor]]:

    client = ros_node.create_client(
        DescribeParameters, f'{node_name}/describe_parameters')

    ready = client.wait_for_service(timeout_sec=1.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = DescribeParameters.Request()
    request.names = parameter_names
    response = await client.call_async(request)
    ros_node.destroy_client(client)

    if response is None or type(response.descriptors) is not list:
        raise RuntimeError('Response value does not exist.')
    if len(response.descriptors) != len(parameter_names):
        raise RuntimeError('Failed to get parameters.')

    descriptors = []
    for descriptor in response.descriptors:
        param_descriptor = _convert_descriptor(descriptor)
        descriptors.append(param_descriptor)
    return descriptors


async def _get_param_names(*, node_name,
                           prefixes: List = [],
                           depth: Optional[int] = None,
                           ros_node: Node) -> ListParametersResult:
    service_name = f'{node_name}/list_parameters'
    client = ros_node.create_client(ListParameters, service_name)
    
    ready = client.wait_for_service(timeout_sec=1.0)
    if not ready:
        ros_node.destroy_client(client)
        raise RuntimeError('Wait for service timed out')

    request = ListParameters.Request()
    if depth is not None:
        request.depth = depth
    request.prefixes = prefixes
    response = await client.call_async(request)
    ros_node.destroy_client(client)
    return response.result




class ROS2ParamAPI:
    def __init__(self, ros_node, *, loop: AbstractEventLoop) -> None:
        self.__ros_node = ros_node
        self.__loop = loop
        self.__logger = launch.logging.get_logger('ROS2ParamAPI')

    async def set(self, node_name: str,
                  parameters: List[Parameter]) -> Optional[List[SetParametersResult]]:
        try:
            return await _set_param(
                node_name=node_name,
                parameters=parameters,
                ros_node=self.__ros_node
            )
        except Exception as e:
            self.__logger.error(e)
        return None

    async def get(self, node_name: str,
                  parameter_names: List[str],
                  describe: bool = True
                  ) -> List[Param]:
        try:
            return await _get_param(
                node_name=node_name,
                parameter_names=parameter_names,
                describe=describe,
                ros_node=self.__ros_node
            )
        except Exception as e:
            self.__logger.error(e)
        return None
        

    async def describe(self, node_name: str,
                       parameter_names: List[str]
                       ) -> List[ParamDescriptor]:
        return await _get_param_descriptor(
            node_name=node_name,
            parameter_names=parameter_names,
            ros_node=self.__ros_node
        )

    async def names(self, node_name: str) -> List[str]:
        result = await _get_param_names(
            node_name=node_name,
            ros_node=self.__ros_node
        )
        return sorted(result.names) if result is not None else []
