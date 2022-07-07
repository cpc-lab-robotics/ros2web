import os.path
from ros2pkg.api.create import _expand_template

def create_template_file(template_file_name, output_directory, output_file_name, template_config):
    if not os.path.exists(template_file_name):
        raise FileNotFoundError('template not found:', template_file_name)
    output_file_path = os.path.join(output_directory, output_file_name)
    _expand_template(template_file_name, template_config, output_file_path)
