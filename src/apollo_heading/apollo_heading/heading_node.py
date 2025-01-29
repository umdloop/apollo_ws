#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
from ahrs.filters import Madgwick
from transforms3d.euler import quat2euler

class HeadingPublisher(Node):
    def __init__(self):
        super().__init__('heading_publisher')
        
        self.declare_parameter('publish_rate', 50.0)  
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.madgwick = Madgwick(
            gyr_noise=0.1,
            acc_noise=0.1,
            frequency=self.publish_rate
        )
        
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        
        self.imu_sub = self.create_subscription(
            Imu,
            'pixhawk_topic_name',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.heading_pub = self.create_publisher(
            Float32,
            'heading',
            10
        )
        
        self.get_logger().info('Heading publisher node initialized')
    
    def imu_callback(self, msg):
        try:
            gyro = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            
            accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            
            self.quaternion = self.madgwick.updateIMU(
                q=self.quaternion,
                gyr=gyro,
                acc=accel
            )
            
            _, _, yaw = quat2euler(self.quaternion)
            
            heading = np.degrees(yaw) % 360.0
            
            heading_msg = Float32()
            heading_msg.data = float(heading)
            self.heading_pub.publish(heading_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HeadingPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()