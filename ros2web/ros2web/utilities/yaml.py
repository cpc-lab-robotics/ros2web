from typing import Dict, Optional
import os.path
import importlib.resources
import yaml

import launch.logging

logger = launch.logging.get_logger(__file__)

def open_yaml_file(package_name, filename)->Dict:
    config = {}
    
    with importlib.resources.path(package_name, "data") as path:
        config_file_path = path.joinpath(filename)
    
    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as yml:
                config = yaml.safe_load(yml)
        except yaml.YAMLError as e:
            logger.error(f'(YAML) :{e}')
    else:
        logger.error(f"File does not exist. ({config_file_path})")
    
    return config