from setuptools import setup

package_name = '@package_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ros2web'],
    zip_safe=True,
    maintainer='@maintainer_name',
    maintainer_email='@maintainer_email',
    description='@package_description',
    license='@package_license',
    entry_points={
@[if extension]@
        'ros2web.@extension.type': [
            '@extension.name = @extension.module_name:@extension.class_name'
        ],
@[end if]@
@[if web_package]@
        'ros2web.@web_package.type': [
            '@web_package.name = @web_package.module_name:@web_package.class_name'
        ],
@[end if]@
    },
    package_data={
        '@package_name': [
            'data/**/*',
        ],
    },
)
