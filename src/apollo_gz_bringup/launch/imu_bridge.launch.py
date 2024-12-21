from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'qos_overrides./imu.publisher.reliability': 'reliable',
                'qos_overrides./imu.publisher.durability': 'volatile',
            }],
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            remappings=[
                ('/imu', 'apollo/imu/data'),
            ]
        ),
    ])