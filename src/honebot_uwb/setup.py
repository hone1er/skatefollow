from glob import glob

from setuptools import find_packages, setup

package_name = 'honebot_uwb'

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
    description='UWB relative-target localisation for honebot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_node = honebot_uwb.uwb_node:main',
        ],
    },
)
