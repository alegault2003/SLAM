import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    pkg_project = FindPackageShare(package="simulation").find("simulation")

    world_path = os.path.join(pkg_project, "worlds", "test")

    gazebo_models_path = os.path.join(pkg_project, "models")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    urdf = os.path.join(get_package_share_directory('simulation'), 'scout-laser.urdf.xacro')
    robot_desc = xacro.process_file(urdf, mappings={'name' : 'laser_robot'}).toxml()


    # ld = LaunchDescription()

    # gazebo_world = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so']
    # )

    # ld.add_action(gazebo_world)

    # return ld

    # # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())
    
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    start_gz = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                           'launch', 'gazebo.launch.py'),)
                ),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
                    arguments=[urdf]),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=["-topic", "/robot_description",  "-entity",  "laser_robot"]),
            ])

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    return ld