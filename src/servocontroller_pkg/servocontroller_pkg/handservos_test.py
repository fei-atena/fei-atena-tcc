import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from adafruit_servokit import ServoKit

class HandservoNode(Node):
    def __init__(self):
        super().__init__('handservo_node')
        self.kit = ServoKit(channels=16)
        self.subscription = self.create_subscription(String, '/hand/command', self.listener_callback, 10)
        self.get_logger().info('HandservoNode started, listening on /hand/command')

    def listener_callback(self, msg):
        command = msg.data.lower()
        if command == 'open':
            self.open_hand()
            self.get_logger().info('Hand opened')
        elif command == 'close':
            self.close_hand()
            self.get_logger().info('Hand closed')
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def open_hand(self):
        self.kit.servo[0].angle = 0
        self.kit.servo[1].angle = 0
        self.kit.servo[2].angle = 0
        self.kit.servo[3].angle = 0
        self.kit.servo[4].angle = 180
        time.sleep(1)

    def close_hand(self):
        self.kit.servo[0].angle = 180
        self.kit.servo[1].angle = 180
        self.kit.servo[2].angle = 180
        self.kit.servo[3].angle = 180
        self.kit.servo[4].angle = 0
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandservoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()