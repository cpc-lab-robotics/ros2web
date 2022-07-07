import os
from ros2pkg.api.create import _create_folder
from ..utilities import create_template_file


def create_config_directory(destination_directory):
    config_directory = os.path.join(destination_directory, ".ros2web")
    if not os.path.isdir(config_directory):
        _create_folder(".ros2web", destination_directory)

    return config_directory
