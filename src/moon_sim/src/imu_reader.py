#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.get_logger().info("✅ IMU Reader started.")
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

    def imu_callback(self, msg):
        self.get_logger().info(f"[IMU] orientation.z = {msg.orientation.z:.3f}, {msg.orientation.y:.3f}, {msg.orientation.x}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuReader())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# This script is a ROS 2 node that reads IMU data and logs the orientation.z value.
# It subscribes to the '/imu' topic and prints the z component of the orientation quaternion
# whenever a new message is received. The node is initialized and spun in the main function.    