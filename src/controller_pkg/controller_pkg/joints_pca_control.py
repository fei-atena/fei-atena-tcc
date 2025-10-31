#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Bool
from adafruit_servokit import ServoKit

class JointsPCAControl(Node):
    def __init__(self):
        super().__init__('joints_pca_control')

        # PCA9685
        self.kit = ServoKit(channels=16)

        # Mapeie cada ação a um canal
        # AJUSTE AQUI conforme a montagem eletrônica:
        self.ch = {
            'left': {
                'shoulder_abd':   0,
                'shoulder_flex':  1,
                'shoulder_rot':   2,
                'elbow_flex':     3,
            },
            'right': {
                'shoulder_abd':   4,
                'shoulder_flex':  5,
                'shoulder_rot':   6,
                'elbow_flex':     7,
            }
        }
        self.hand_ch = {'left': 8, 'right': 9}  # se tiver servo da mão num canal dedicado

        # Limites físicos de ângulo enviados ao ServoKit (proteção)
        self.servo_min, self.servo_max = 0, 110

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions lado ESQUERDO
        self.create_subscription(Int32, '/left/shoulder_flex', self.mk_cb('left','shoulder_flex'), qos)
        self.create_subscription(Int32, '/left/shoulder_abd',  self.mk_cb('left','shoulder_abd'),  qos)
        self.create_subscription(Int32, '/left/shoulder_rot',  self.mk_cb('left','shoulder_rot'),  qos)
        self.create_subscription(Int32, '/left/elbow_flex',    self.mk_cb('left','elbow_flex'),    qos)
        self.create_subscription(Bool,  '/left/hand_open',     self.mk_hand_cb('left'),            qos)

        # Subscriptions lado DIREITO
        self.create_subscription(Int32, '/right/shoulder_flex', self.mk_cb('right','shoulder_flex'), qos)
        self.create_subscription(Int32, '/right/shoulder_abd',  self.mk_cb('right','shoulder_abd'),  qos)
        self.create_subscription(Int32, '/right/shoulder_rot',  self.mk_cb('right','shoulder_rot'),  qos)
        self.create_subscription(Int32, '/right/elbow_flex',    self.mk_cb('right','elbow_flex'),    qos)
        self.create_subscription(Bool,  '/right/hand_open',     self.mk_hand_cb('right'),            qos)

        self.initialize_servos()
        self.get_logger().info('JointsPCAControl ouvindo tópicos anatômicos.')

    def initialize_servos(self):
        for side in self.ch:
            for action, channel in self.ch[side].items():
                self.safe_set(channel, 0)
        for side, channel in self.hand_ch.items():
            self.safe_set(channel, 0)

    def clamp(self, a):
        try:
            a = int(a)
        except Exception:
            a = 0
        return max(self.servo_min, min(self.servo_max, a))

    def safe_set(self, channel, angle):
        angle = self.clamp(angle)
        try:
            self.kit.servo[channel].angle = angle
        except Exception as e:
            self.get_logger().error(f'Falha no canal {channel}: {e}')

    def mk_cb(self, side, action):
        channel = self.ch[side][action]
        def _cb(msg: Int32):
            self.safe_set(channel, msg.data)
            self.get_logger().info(f'{side}.{action} → ch{channel} = {self.clamp(msg.data)}')
        return _cb

    def mk_hand_cb(self, side):
        channel = self.hand_ch[side]
        def _cb(msg: Bool):
            # Exemplo simples: aberto=110°, fechado=0°
            target = 110 if msg.data else 0
            self.safe_set(channel, target)
            self.get_logger().info(f'{side}.hand_open={msg.data} → ch{channel} = {target}')
        return _cb

def main(args=None):
    rclpy.init(args=args)
    node = JointsPCAControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
