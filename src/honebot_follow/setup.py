from glob import glob

from setuptools import find_packages, setup

package_name = 'honebot_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hone1er',
    maintainer_email='hone1er@todo.todo',
    description='Follow-me controller for honebot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower = honebot_follow.follower:main',
            'pose_to_target = honebot_follow.pose_to_target:main',
        ],
    },
)
