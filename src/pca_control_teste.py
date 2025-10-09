#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from adafruit_servokit import ServoKit

class PCAControlTestNode(Node):
    def __init__(self):
        super().__init__('pca_control_test_node')

        self.kit = ServoKit(channels=16)

        # Set all servos to initial position 0
        self.initialize_servos()

        # Subscriber
        self.motor_sub = self.create_subscription(
            Int32, 
            "/motor_values", 
            self.motor_callback, 
            10  # Default QoS depth
        )

    def initialize_servos(self):
        for channel in range(16):
            self.kit.servo[channel].angle = 0

    def motor_callback(self, msg: Int32):
        value = msg.data
        if 0 <= value <= 180:
            for channel in range(16):
                self.kit.servo[channel].angle = value
        else:
            self.get_logger().warn(f"Received invalid motor value: {value}. Must be between 0 and 180.")

def main(args=None):
    rclpy.init(args=args)
    node = PCAControlTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()