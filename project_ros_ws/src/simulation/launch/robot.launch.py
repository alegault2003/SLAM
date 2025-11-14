import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    bed_urdf = os.path.join(get_package_share_directory('project'), 'bed.urdf')
    robot_urdf = os.path.join(get_package_share_directory('project'), 'scout-camera.urdf.xacro')
    robot_desc = xacro.process_file(robot_urdf, mappings={'name' : 'camera_robot'}).toxml()

    return LaunchDescription([
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                           'launch', 'gazebo.launch.py'),
             )
        ),

        Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
             arguments=[robot_urdf, bed_urdf]),
        Node(
             package='gazebo_ros',
             executable='spawn_entity.py',
             name='urdf_spawner',
             output='screen',
             arguments=["-topic", "/robot_description",  "-entity",  "camera_robot"]),
    ])

