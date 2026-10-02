from setuptools import setup
import os
from glob import glob

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='PX4 Offboard Control with Teleop',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_offboard_control.py = px4_offboard.position_offboard_control:main', 
            'teleop_keyboard.py = px4_offboard.teleop_keyboard:main',
            'velocity_offboard_control.py = px4_offboard.velocity_offboard_control:main', 
            'teleop_joystick.py = px4_offboard.teleop_joystick:main',
        ],
    },
)