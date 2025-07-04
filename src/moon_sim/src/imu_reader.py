#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.get_logger().info("✅ IMU Reader started.")
        self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        self.orientation_z = 0.0
        self.orientation_y = 0.0
        self.orientation_x = 0.0
    def imu_callback(self, msg):
        self.orientation_z = msg.orientation.z
        self.orientation_y = msg.orientation.y
        self.orientation_x = msg.orientation.x

        #self.get_logger().info(f"[IMU] orientation.z = {msg.orientation.z:.3f}, {msg.orientation.y:.3f}, {msg.orientation.x}")

# This script is a ROS 2 node that reads IMU data and logs the orientation.z value.
# It subscribes to the '/imu' topic and prints the z component of the orientation quaternion
# whenever a new message is received. The node is initialized and spun in the main function.    
def main():
    rclpy.init()
    imusensor = ImuReader()
    while True:
        rclpy.spin_once(imusensor)
        print(f"x: {imusensor.orientation_x} y: {imusensor.orientation_y} z:{imusensor.orientation_z}" )
    rclpy.shutdown()

if __name__ == '__main__':
    main()