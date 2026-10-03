from glob import glob
import os

from setuptools import setup

package_name = 'mini_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.txt')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nyankowan',
    maintainer_email='nyankowan5@gmail.com',
    description='Minimal 2D autonomous navigation stack for learning ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = mini_nav.simulator_node:main',
            'planner = mini_nav.planner_node:main',
            'controller = mini_nav.controller_node:main',
        ],
    },
)
