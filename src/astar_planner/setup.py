from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'astar_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ana',
    maintainer_email='apmorale@espol.edu.ec',
    description='Package for A* navigationn and control for F1TENT robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner_node = astar_planner.astar_planner_node:main',
            'controller = astar_planner.controller:main',
            'planner = astar_planner.planner:main',
        ],
    },
)
