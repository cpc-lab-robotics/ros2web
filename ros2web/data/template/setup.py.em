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
        'ros2web.package': [
            '@web_package_name = @package_name:@class_name'
        ],
    },
    package_data={
        '@package_name': [
            'data/**/*',
        ],
    },
)
