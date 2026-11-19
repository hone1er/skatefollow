from glob import glob

from setuptools import find_packages, setup

package_name = 'honebot_remote_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hone1er',
    maintainer_email='hone1er@todo.todo',
    description='Handheld ESP32 remote -> ROS 2 Twist, to drive turtlesim like the robot would move.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remote_bridge = honebot_remote_bridge.remote_bridge:main',
        ],
    },
)
