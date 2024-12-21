from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]




def generate_launch_description():
    # Directories
    pkg_apollo_description = get_package_share_directory(
        'apollo_description')

    # Paths
    robot_description_launch = PathJoinSubstitution(
        [pkg_apollo_description, 'launch', 'robot_description.launch.py'])
    # Launch configurations
    namespace = LaunchConfiguration('namespace')

    robot_name = 'apollo'

    # Spawn robot slightly closer to the floor to reduce the drop
    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),
        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))]
        ),

        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', '0.0',
                       '-y', '0.0',
                       '-z', '3.0',
                       '-Y', '0.0',
                       '-topic', 'robot_description'],
            output='screen'
        ),
    ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld