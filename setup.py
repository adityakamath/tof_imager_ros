import os
from glob import glob
from setuptools import setup

package_name = 'vl53l5cx_ros'

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
    description='ROS 2 Node for the VL53L5CX ToF Imager',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_imager_publisher = vl53l5cx_ros.tof_imager_publisher:main',
            'tof_imager_node = vl53l5cx_ros.tof_imager_node:main',
        ],
    },
)
