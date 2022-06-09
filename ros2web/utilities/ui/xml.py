from typing import Dict
from typing import Union

import os
import os.path
import re
import time
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element
from ..key_generator import KeyGenerator


def parse_xml(*, key_generator:KeyGenerator, element: Element) -> Union[Dict, str]:
    
    el_data = {'name': element.tag}
    key = key_generator.get_key()

    props = {'key': key}
    for k, v in element.items():
        m = re.fullmatch(r'^{(.+)}$', v)
        if m is not None:
            value = re.sub(r"([{|,])\s+(\S+):", r'\1\2:', m[1])
            value = re.sub(r":\s+", ':', value)
            value = re.sub(r"(\S)\s+}", r'\1}', value)
            value = re.sub(r",}", r'}', value)
            value = re.sub(r"{(\w|[^\']\S+?[^\']):", r"{'\1':", value)
            value = re.sub(r",(\w|[^\']\S+?[^\']):", r",'\1':", value)
            value = value.replace('"', '\"')
            value = value.replace("'", '"')
            props[k] = value
        else:
            props[k] = v

    if len(props) > 0:
        el_data['props'] = props

    children = []
    for child in element:
        d = parse_xml(key_generator=key_generator,
                      element=child)
        children.append(d)
    if len(children) > 0:
        el_data['children'] = children
    else:
        el_data['children'] = element.text
    return el_data


def load_xml(*, key_generator: str, xml_data: str):

    _, ext = os.path.splitext(xml_data)
    if ext != "":
        tree = ET.parse(xml_data)
        root = tree.getroot()
    else:
        root = ET.fromstring(xml_data)
    
    return parse_xml(key_generator=key_generator, element=root)


def create_element(*, name, xml):
    data = None
    if xml is not None:
        salt=f"{name}-{int(time.time())}"
        key_gen = KeyGenerator(salt=salt)
        data = load_xml(key_generator=key_gen,
                               xml_data=xml)

    return data
