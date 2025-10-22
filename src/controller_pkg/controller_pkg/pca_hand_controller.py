#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
import time
from adafruit_servokit import ServoKit

class HandservoNode(Node):
    def __init__(self):
        super().__init__('handservo_node')

        self.kit = ServoKit(channels=16)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # garante entrega (é estado, não sensor)
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # guarda o último valor (latched)
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.right_hand_sub = self.create_subscription(
            Bool, 
            "/right/hand_joint", 
            self.right_hand_callback, 
            qos_profile
        )

    def right_hand_callback(self, hand_state: Bool):
        if hand_state.data:
            self.close_hand()
        else:
            self.open_hand()

    def open_hand(self):
        self.kit.servo[0].angle = 0
        self.kit.servo[1].angle = 0
        self.kit.servo[2].angle = 0
        self.kit.servo[3].angle = 0
        self.kit.servo[4].angle = 180

    def close_hand(self):
        self.kit.servo[0].angle = 180
        self.kit.servo[1].angle = 180
        self.kit.servo[2].angle = 180
        self.kit.servo[3].angle = 180
        self.kit.servo[4].angle = 0

def main(args=None):
    rclpy.init(args=args)
    node = HandservoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()