from glob import glob

from setuptools import find_packages, setup

package_name = 'honebot_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/netplan', glob('netplan/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hone1er',
    maintainer_email='hone1er@todo.todo',
    description='GoPro / USB camera input, person detection and camera pan aiming for honebot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = honebot_camera.camera_node:main',
            'camera_aim = honebot_camera.camera_aim:main',
            'gopro_ctl = honebot_camera.gopro:main',
        ],
    },
)
