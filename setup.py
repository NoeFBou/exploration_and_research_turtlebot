from setuptools import setup
import os
from glob import glob

package_name = 'tb3_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sacha',
    maintainer_email='sacha@todo.todo',
    description='Projet exploration Turtlebot3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor = tb3_autonomy.mission_supervisor:main',
            'supervisor_node = tb3_autonomy.supervisor_node:main',
            'camera_ai = tb3_autonomy.camera_processor:main',
        ],
    },
)