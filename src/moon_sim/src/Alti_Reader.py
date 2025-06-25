#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from your_package.msg import Altimeter  # Replace with actual message

class AltimeterReader(Node):
    def __init__(self):
        super().__init__('altimeter_reader')
        self.get_logger().info("📡 Altimeter Reader started.")
        self.create_subscription(Altimeter, '/altimeter', self.altimeter_callback, 10)

    def altimeter_callback(self, msg):
        self.get_logger().info(
            f"[Altimeter] height: {msg.vertical_position:.2f} m, "
            f"velocity: {msg.vertical_velocity:.2f} m/s, "
            f"reference: {msg.vertical_reference:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AltimeterReader())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
