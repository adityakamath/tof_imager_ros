import os
from glob import glob
from setuptools import setup

package_name = 'tof_imager_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'vl53l5cx'],
    zip_safe=True,
    maintainer='Aditya Kamath',
    maintainer_email='adityakamath@live.com',
    description='ROS 2 package for the VL53L5CX and VL53L7CX ToF Imagers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_imager_publisher = tof_imager_ros.tof_imager_publisher:main',
            'tof_imager_node = tof_imager_ros.tof_imager_node:main',
        ],
    },
)
