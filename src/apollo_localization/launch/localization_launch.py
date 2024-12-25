from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for additional configuration
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Get the path to the configuration package
    pkg_apollo_localization = FindPackageShare('apollo_localization')

    # Define the path to the EKF configuration file
    ekf_config_path = PathJoinSubstitution([
        pkg_apollo_localization,
        'config',
        'ekf.yaml'
    ])

    # Create the EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
    )

    return LaunchDescription([
        use_sim_time,
        ekf_node
    ])