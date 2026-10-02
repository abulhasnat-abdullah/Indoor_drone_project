import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'odom_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abd',
    maintainer_email='abd@example.com',
    description=(
        'Bridges PX4 vehicle_odometry <-> ROS 2 odometry/TF, and '
        'slam_toolbox pose -> PX4 external vision odometry. Also '
        'provides 3D pose/mapping from the 2D lidar and a frontier explorer.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_bridge_node = odom_bridge.odom_bridge_node:main',
            'vision_odom_bridge = odom_bridge.vision_odom_bridge:main',
            'pose_3d_node = odom_bridge.pose_3d_node:main',
            'scan_3d_mapper = odom_bridge.scan_3d_mapper:main',
            'frontier_explorer = odom_bridge.frontier_explorer:main',
            'reactive_avoidance = odom_bridge.reactive_avoidance:main',
        ],
    },
)
