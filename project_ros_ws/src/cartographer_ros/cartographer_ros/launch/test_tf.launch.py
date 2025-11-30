from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # base_link -> points2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_points2',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'points2']
        ),
    ])
