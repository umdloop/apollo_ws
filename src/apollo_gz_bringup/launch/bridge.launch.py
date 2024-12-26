from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'qos_overrides./imu.publisher.reliability': 'reliable',
                'qos_overrides./imu.publisher.durability': 'reliable',
            }],
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            remappings=[
                ('/imu', 'apollo/imu'),
            ]
        ),
        
        # GPS Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gps_bridge',
            output='screen',
            parameters=[{
                'qos_overrides./gps/fix.publisher.reliability': 'reliable',
                'qos_overrides./gps/fix.publisher.durability': 'reliable',
            }],
            arguments=[
                '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            ],
            remappings=[
                ('/gps/fix', 'apollo/gps/fix'),
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_image_bridge',
            output='screen',
            arguments=[
                '/world/apollo_world/model/apollo/link/chassis/sensor/zed2_depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            remappings=[
                ('/world/apollo_world/model/apollo/link/chassis/sensor/zed2_depth_camera/depth_image', 'apollo/zed2_depth_camera/depth_image'),
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_points_bridge',
            output='screen',
            arguments=[
                '/world/apollo_world/model/apollo/link/chassis/sensor/zed2_depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/world/apollo_world/model/apollo/link/chassis/sensor/zed2_depth_camera/depth_image/points', 'apollo/zed2_depth_camera/depth_image/points'),
            ]
        ),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
                       name='clock_bridge',
                       output='screen',
                       arguments=[
                           '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                       ])
    ])