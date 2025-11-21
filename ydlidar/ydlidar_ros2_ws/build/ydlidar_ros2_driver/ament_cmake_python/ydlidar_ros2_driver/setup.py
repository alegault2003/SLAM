from setuptools import find_packages
from setuptools import setup

setup(
    name='ydlidar_ros2_driver',
    version='1.0.2',
    packages=find_packages(
        include=('ydlidar_ros2_driver', 'ydlidar_ros2_driver.*')),
)
