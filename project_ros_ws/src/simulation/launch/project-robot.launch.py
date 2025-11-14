import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pass
    urdf = os.path.join(get_package_share_directory('simulation'), 'scout-laser.urdf.xacro')
    robot_desc = xacro.process_file(urdf, mappings={'name' : 'laser_robot'}).toxml()
    # world_path = os.path.join(get_package_share_directory('simulation'), "worlds", "test")