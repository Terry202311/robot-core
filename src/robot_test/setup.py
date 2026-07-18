from setuptools import find_packages, setup

package_name = 'robot_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Low-speed motion test tools for the mecanum robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_test = robot_test.motion_test:main',
            'forward_test = robot_test.forward_test:run',
            'backward_test = robot_test.backward_test:run',
            'left_test = robot_test.left_test:run',
            'right_test = robot_test.right_test:run',
            'rotate_left_test = robot_test.rotate_left_test:run',
            'rotate_right_test = robot_test.rotate_right_test:run',
        ],
    },
)
