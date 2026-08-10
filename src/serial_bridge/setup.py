import os
from glob import glob
from setuptools import setup
package_name='serial_bridge'
setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name
    ],

    install_requires=[
        'setuptools',
        'pyserial'
    ],


    entry_points={
        'console_scripts':[
            'serial_bridge_node = serial_bridge.serial_bridge_node:main'
        ]
    },

    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/serial_bridge']
    ),
    (
        'share/serial_bridge',
        ['package.xml']
    ),
    (
        os.path.join(
            'share',
            'serial_bridge',
            'launch'
        ),
        glob('launch/*.launch.py')
    ),

     ]
)