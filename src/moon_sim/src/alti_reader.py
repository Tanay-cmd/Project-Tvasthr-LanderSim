#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter  

class AltimeterReader(Node):
    def __init__(self):
        super().__init__('altimeter_reader')
        self.get_logger().info("📡 Altimeter Reader started.")
        self.create_subscription(Altimeter, '/altimeter', self.altimeter_callback, 10)

        self.vertical_position = 0.000000
        self.vertical_velocity = 0.000000
        self.vertical_reference = 0.00000
        
    def altimeter_callback(self, msg):
        self.vertical_position = msg.vertical_position
        self.vertical_velocity = msg.vertical_velocity
        self.vertical_reference = msg.vertical_reference
    

# def main():
#     rclpy.init()
#     altimeter = AltimeterReader()
#     while True:
#         rclpy.spin_once(altimeter)
#         print(f"x: {altimeter.orientation_x} y: {altimeter.orientation_y} z:{altimeter.orientation_z}" )
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()     

