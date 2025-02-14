"""
@file       setup.py

@author     Silvia R. Alcaraz
@version    1.0

@copyright  Copyright (C) 2025 
"""

from setuptools import find_packages, setup

package_name = 'lidar_processing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', ['launch/lidar_processor_launch.py']), 
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'open3d', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='silviaralcaraz',
    maintainer_email='silvia.alcaraz@usc.es',
    description='Basic lidar processing',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_processing.main:main',
        ],
    },
)
