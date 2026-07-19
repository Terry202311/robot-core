from setuptools import find_packages, setup

package_name = 'wt901_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/wt901.launch.py']
        ),
        (
            'share/' + package_name + '/config',
            ['config/wt901.yaml']
        ),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='ROS2 driver for WT901 IMU',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wt901_node = wt901_driver.wt901_node:main',
        ],
    },
)
