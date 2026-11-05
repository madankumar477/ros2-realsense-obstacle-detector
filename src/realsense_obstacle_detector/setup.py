# Standalone packaging metadata updated by E MADAN KUMAR (2026).
from setuptools import setup
import os
from glob import glob

package_name = 'realsense_obstacle_detector'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='E MADAN KUMAR',
    maintainer_email='madan.kumar@atimotors.com',
    description='ROS 2 RealSense D435i depth-based close-range obstacle detection',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'realsense_obstacle_detector = '
            'realsense_obstacle_detector.realsense_obstacle_detector:main',
        ],
    },
)
