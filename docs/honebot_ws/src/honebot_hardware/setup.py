from glob import glob

from setuptools import find_packages, setup

package_name = 'honebot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hone1er',
    maintainer_email='hone1er@todo.todo',
    description='ROS 2 <-> ESP32 serial bridge for honebot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32_bridge = honebot_hardware.esp32_bridge:main',
        ],
    },
)
