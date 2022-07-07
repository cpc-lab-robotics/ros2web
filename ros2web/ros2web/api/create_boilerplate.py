from tkinter import Pack
from typing import Dict, NamedTuple
import getpass
import os
import shutil
import subprocess
import pkg_resources

import launch.logging
from catkin_pkg.package import Dependency, Export, Person

from ..utilities import create_template_file

logger = launch.logging.get_logger(os.path.basename(__file__))


class PackageDirectory(NamedTuple):
    root: str
    resource: str
    src: str
    data: str


class PluginInfo(NamedTuple):
    type: str
    name: str
    module_name: str
    class_name: str


def _create_maintainer() -> Person:
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

    return maintainer


def _create_plugin_info(*, package_name, plugin_name, plugin_type: str) -> PluginInfo:
    name = plugin_name.lower()
    package_class_name = f"ROS2Web{plugin_type.capitalize()}"

    module_name = f"{package_name}.{plugin_type}"

    return PluginInfo(
        type=plugin_type,
        name=name,
        class_name=package_class_name,
        module_name=module_name
    )


def _create_directory(package_name: str, destination_directory: str):
    directory = os.path.join(destination_directory, package_name)
    if os.path.exists(directory):
        raise RuntimeError('\nAborted!\nThe directory already exists: ' + directory + '\nEither ' +
                           'remove the directory or choose a different destination directory or package name')

    resource_directory = os.path.join(directory, 'resource')
    src_directory = os.path.join(directory, package_name)
    data_directory = os.path.join(src_directory, 'data')

    os.mkdir(directory)
    os.mkdir(resource_directory)
    os.mkdir(src_directory)
    os.mkdir(data_directory)

    return PackageDirectory(
        root=directory,
        resource=resource_directory,
        src=src_directory,
        data=data_directory
    )


def _create_python_files(
        package_name: str, package_directory: PackageDirectory,
        extension: PluginInfo, web_package: PluginInfo, maintainer: Person):

    template_path = pkg_resources.resource_filename('ros2web', 'data/template')
    python_template_path = os.path.join(template_path, 'python')

    package_license = 'TODO: License declaration'
    package_description = 'TODO: Package description'

    create_template_file(os.path.join(python_template_path, 'package.xml.em'), package_directory.root,
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

    create_template_file(os.path.join(python_template_path, 'setup.py.em'), package_directory.root,
                         'setup.py', {
                             'package_name': package_name,
                             'extension': extension,
                             'web_package': web_package,
                             'maintainer_email': maintainer.email,
                             'maintainer_name': maintainer.name,
                             'package_license': package_license,
                             'package_description': package_description,
    })

    create_template_file(os.path.join(python_template_path, 'setup.cfg.em'), package_directory.root,
                         'setup.cfg', {
                             'package_name': package_name,
    })

    create_template_file(os.path.join(python_template_path, 'resource_file.em'),
                         package_directory.resource, package_name, {})

    create_template_file(os.path.join(python_template_path, '__init__.py.em'),
                         package_directory.src, '__init__.py', {})

    if extension:
        create_template_file(os.path.join(python_template_path, f'{extension.type}.py.em'),
                             package_directory.src,
                             f'{extension.type}.py', {
            'package_name': package_name,
            'plugin_name': extension.name,
        })

    if web_package:
        create_template_file(os.path.join(python_template_path, f'{web_package.type}.py.em'),
                             package_directory.src,
                             f'{web_package.type}.py', {
            'package_name': package_name,
            'plugin_name': web_package.name,
        })

    create_template_file(os.path.join(template_path, 'config.yml.em'),
                         package_directory.data, 'config.yml',
                         {'extension': extension is not None,
                          'package': web_package is not None
                          })


def _create_web_files(package_name: str, extension: PluginInfo, package_directory: PackageDirectory):

    template_path = pkg_resources.resource_filename('ros2web', 'data/template')
    web_template_path = os.path.join(template_path, 'web')

    web_directory = os.path.join(package_directory.root, 'web')
    web_src_directory = os.path.join(web_directory, 'src')
    os.mkdir(web_directory)
    os.mkdir(web_src_directory)

    create_template_file(os.path.join(template_path, 'COLCON_IGNORE.em'),
                         web_directory, 'COLCON_IGNORE', {})
    create_template_file(os.path.join(
        web_template_path, 'index.html.em'), web_directory, 'index.html', {})

    create_template_file(os.path.join(web_template_path, 'package.json.em'), web_directory,
                         'package.json', {'package_name': package_name})
    create_template_file(os.path.join(web_template_path, 'tsconfig.json.em'),
                         web_directory, 'tsconfig.json', {})
    create_template_file(os.path.join(web_template_path, 'webpack.config.js.em'),
                         web_directory, 'webpack.config.js', {})
    create_template_file(os.path.join(web_template_path, 'widget-config.js.em'),
                         web_directory, 'widget-config.js', {
                             'plugin_name': extension.name,
                             'package_name': package_name})
    create_template_file(os.path.join(web_template_path, 'bootstrap.tsx.em'),
                         web_src_directory, 'bootstrap.tsx', {})
    create_template_file(os.path.join(web_template_path, 'index.ts.em'),
                         web_src_directory, 'index.ts', {})
    create_template_file(os.path.join(web_template_path, 'Widget.tsx.em'),
                         web_src_directory, 'Widget.tsx', {
                             'plugin_name': extension.name
    })


def _create_files(package_name: str, package_directory: PackageDirectory,
                  web_package: PluginInfo, extension: PluginInfo, maintainer: Person):

    _create_python_files(package_name, package_directory,
                         extension, web_package, maintainer)

    if extension:
        _create_web_files(package_name, extension, package_directory)


def create_package(name: str, destination_directory, type: Dict):
    web_package = None
    extension = None
    package_directory_root = None
    try:
        package_name = f"ros2web_{name}"

        package_directory = _create_directory(
            package_name, destination_directory)

        package_directory_root = package_directory.root

        maintainer = _create_maintainer()
        if type['package']:
            web_package = _create_plugin_info(
                package_name=package_name,
                plugin_name=name, plugin_type='package')

        if type['extension']:
            extension = _create_plugin_info(
                package_name=package_name,
                plugin_name=name, plugin_type='extension')

        _create_files(package_name, package_directory,
                      web_package, extension, maintainer)

        print(f'Successfully created {package_name}.')
    except Exception as e:
        logger.error(e)
        if package_directory_root and os.path.exists(package_directory_root):
            shutil.rmtree(package_directory_root)
