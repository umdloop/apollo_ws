from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
]

def generate_launch_description():
    # Directories
    pkg_apollo_gz_bringup = get_package_share_directory('apollo_gz_bringup')


    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_apollo_gz_bringup, 'launch', 'gz.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_apollo_gz_bringup, 'launch', 'apollo_spawn.launch.py'])
    bridge_launch = PathJoinSubstitution(
        [pkg_apollo_gz_bringup, 'launch', 'bridge.launch.py'])

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch])
    )

    # Launch robot spawner
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz'))]
    )

    # Launch IMU bridge
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bridge_launch])
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    ld.add_action(bridge)
    return ld