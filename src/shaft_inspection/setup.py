from setuptools import setup
import os
from glob import glob

package_name = 'shaft_inspection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'deploy'), glob('deploy/*')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abul Hasnat Abdullah',
    maintainer_email='203359841+abulhasnat-abdullah@users.noreply.github.com',
    description='Autonomous vertical-shaft inspection for a PX4 drone (lidar only).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'shaft_perception = shaft_inspection.shaft_perception_node:main',
            'shaft_mission    = shaft_inspection.shaft_mission_node:main',
            'shaft_mapper     = shaft_inspection.shaft_mapper_node:main',
            'shaft_preflight  = shaft_inspection.preflight_check:main',
            'shaft_dashboard  = shaft_inspection.shaft_dashboard_node:main',
            'shaft_mount_check = shaft_inspection.lidar_mount_check:main',
        ],
    },
)
