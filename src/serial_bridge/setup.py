from setuptools import setup
from glob import glob
import os

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenRobotPlatform',
    maintainer_email='opensource@openrobotplatform.org',
    description='ROS2 serial bridge for Arduino Mega2560',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = serial_bridge.serial_bridge_node:main',
        ],
    },
)
