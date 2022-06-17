import getpass
import os
import shutil
import subprocess
import pkg_resources

from catkin_pkg.package import Dependency, Export, Person
from ros2pkg.api.create import _expand_template, _create_folder


def _create_template_file(template_file_name, output_directory, output_file_name, template_config):
    template_path = pkg_resources.resource_filename(
        'ros2web', 'data/template/' + template_file_name)
    if not os.path.exists(template_path):
        raise FileNotFoundError('template not found:', template_path)
    output_file_path = os.path.join(output_directory, output_file_name)
    _expand_template(template_path, template_config, output_file_path)


def _create_web_package(web_package_name, destination_directory):

    package_name = f"ros2web_{web_package_name}"
    package_license = 'TODO: License declaration'
    package_description = 'TODO: Package description'

    maintainer = Person(getpass.getuser())
    git = shutil.which('git')
    if git is not None:
        p = subprocess.Popen(
            [git, 'config', '--global', 'user.email'],
            stdout=subprocess.PIPE)
        resp = p.communicate()
        email = resp[0].decode().rstrip()
        if email:
            maintainer.email = email
    if not maintainer.email:
        maintainer.email = maintainer.name + '@todo.todo'

    package_directory = _create_folder(package_name, destination_directory)

    _create_template_file('package.xml.em', package_directory,
                          'package.xml', {
                              'package_format': 3,
                              'package_name': package_name,
                              'package_description': package_description,
                              'maintainer_email': maintainer.email,
                              'maintainer_name': maintainer.name,
                              'package_license': package_license,
                              'dependencies': [Dependency(dep) for dep in ['ros2web']],
                              'exports': [Export('build_type', content='ament_python')],
                          })

    resource_directory = _create_folder('resource', package_directory)
    _create_template_file('resource_file.em', resource_directory,
                          package_name, {})



    new_web_package_name = ""
    for name in web_package_name.split('_'):
        new_web_package_name += name.capitalize()

    class_name = f"ROS2Web{new_web_package_name}"
    _create_template_file('setup.py.em', package_directory,
                          'setup.py', {
                              'package_name': package_name,
                              'class_name': class_name,
                              'maintainer_email': maintainer.email,
                              'maintainer_name': maintainer.name,
                              'package_license': package_license,
                              'package_description': package_description,
                              'web_package_name': web_package_name,
                          })

    _create_template_file('setup.cfg.em', package_directory,
                          'setup.cfg', {
                              'package_name': package_name,
                          })

    inheritance_name = f"{class_name}(WebPackage)"
    package_src_directory = _create_folder(package_name, package_directory)
    _create_template_file('__init__.py.em', package_src_directory,
                          '__init__.py', {
                              'package_name': package_name,
                              'class_name': class_name,
                              'inheritance_name': inheritance_name,
                              'web_package_name': web_package_name,
                          })

    package_data_directory = _create_folder('data', package_src_directory)

    _create_template_file('config.yml.em', package_data_directory,
                          'config.yml', {})

    _create_folder('public', package_data_directory)


def _create_config_directory(destination_directory: str, config_directory_name: str):
    config_directory = _create_folder(
        config_directory_name, destination_directory)
    _create_template_file('COLCON_IGNORE.em',
                          config_directory, 'COLCON_IGNORE', {})


def create_web_package(web_package_name:str, destination_directory):

    web_package_name = web_package_name.lower()
    
    web_package_path = os.path.join(
        destination_directory, web_package_name)

    if os.path.exists(web_package_path):
        return '\nAborted!\nThe directory already exists: ' + web_package_path + '\nEither ' + \
            'remove the directory or choose a different destination directory or package name'

    _create_web_package(web_package_name, destination_directory)


def create_config_directory(destination_directory):
    config_directory = os.path.join(destination_directory, "web")
    if not os.path.isdir(config_directory):
        _create_config_directory(destination_directory, "web")

    return config_directory
