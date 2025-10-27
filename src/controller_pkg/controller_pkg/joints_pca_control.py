#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
from adafruit_servokit import ServoKit

"""
Node para controlar servos via PCA9685 (ServoKit). Subscreve aos tópicos publicados
por `processamento_node.py` e posiciona os servos correspondentes.

Assunções/Configuração inicial (ajustar se necessário):
- ShoulderLeft yaw -> canal 0 (range servo: 0..90)
- ShoulderLeft pitch -> canal 2 (range servo: 0..110)
- Left/Right hand -> canais opcionais (definir conforme sua placa)

Implementação: callbacks recebem Int32 (0-180 ou valores mapeados pelo processamento)
e definem o ângulo do servo com clamp em [0, 180].
"""


class JointsPCAControl(Node):
    def __init__(self):
        super().__init__('joints_pca_control')

        # PCA/ServoKit
        self.kit = ServoKit(channels=16)
        # Mapear canais (ajuste conforme sua fiação)
        self.SHOULDER_LEFT_ROLL_CH = 0
        self.SHOULDER_LEFT_PITCH_CH = 1
        self.SHOULDER_LEFT_YAW_CH = 2
        self.ELBOW_LEFT_PITCH_CH = 3

        self.SHOULDER_RIGHT_ROLL_CH = 4
        self.SHOULDER_RIGHT_PITCH_CH = 5
        self.SHOULDER_RIGHT_YAW_CH = 6
        self.ELBOW_RIGHT_PITCH_CH = 7

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions: os tópicos devem existir no processamento_node.py
        self.create_subscription(Int32, '/left/shoulder_pitch', self.left_shoulder_pitch_cb, qos_profile)
        self.create_subscription(Int32, '/left/shoulder_yaw',   self.left_shoulder_yaw_cb,   qos_profile)
        # Subscriptions para os cotovelos
        self.create_subscription(Int32, '/left/elbow_pitch', self.left_elbow_pitch_cb, qos_profile)
        self.create_subscription(Int32, '/right/elbow_pitch', self.right_elbow_pitch_cb, qos_profile)
        # Subscriptions para os ombros (incluindo roll)
        self.create_subscription(Int32, '/left/shoulder_roll', self.left_shoulder_roll_cb, qos_profile)
        self.create_subscription(Int32, '/right/shoulder_pitch', self.right_shoulder_pitch_cb, qos_profile)
        self.create_subscription(Int32, '/right/shoulder_yaw', self.right_shoulder_yaw_cb, qos_profile)
        self.create_subscription(Int32, '/right/shoulder_roll', self.right_shoulder_roll_cb, qos_profile)

        # inicializar servos em posição segura (0)
        self.initialize_servos()

        self.get_logger().info('Joints PCA Control iniciado.')

    def initialize_servos(self):
        # Inicializa apenas os canais que vamos controlar
        for ch in (self.SHOULDER_LEFT_YAW_CH, self.SHOULDER_LEFT_PITCH_CH, self.SHOULDER_LEFT_ROLL_CH,
                   self.SHOULDER_RIGHT_PITCH_CH, self.SHOULDER_RIGHT_YAW_CH, self.SHOULDER_RIGHT_ROLL_CH,
                   self.ELBOW_LEFT_PITCH_CH, self.ELBOW_RIGHT_PITCH_CH):
            try:
                # Se canal não suportar ângulo, ignore
                self.kit.servo[ch].angle = 0
            except Exception:
                # log e continuar
                self.get_logger().debug(f'Não foi possível inicializar servo no canal {ch} (pode não estar conectado)')

    def clamp_angle(self, angle: int) -> int:
        try:
            a = int(angle)
        except Exception:
            return 0
        return max(0, min(180, a))

    def left_shoulder_pitch_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        # pitch servo tem faixa física menor (0..110) mas ServoKit aceita 0..180; mantenha clamp
        try:
            self.kit.servo[self.SHOULDER_LEFT_PITCH_CH].angle = angle
            self.get_logger().info(f'Set left shoulder pitch (ch {self.SHOULDER_LEFT_PITCH_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir pitch: {e}')

    def left_shoulder_yaw_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.SHOULDER_LEFT_YAW_CH].angle = angle
            self.get_logger().info(f'Set left shoulder yaw (ch {self.SHOULDER_LEFT_YAW_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir yaw: {e}')

    def left_elbow_pitch_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.ELBOW_LEFT_PITCH_CH].angle = angle
            self.get_logger().info(f'Set left elbow pitch (ch {self.ELBOW_LEFT_PITCH_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir pitch do cotovelo esquerdo: {e}')

    def right_elbow_pitch_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.ELBOW_RIGHT_PITCH_CH].angle = angle
            self.get_logger().info(f'Set right elbow pitch (ch {self.ELBOW_RIGHT_PITCH_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir pitch do cotovelo direito: {e}')

    def left_shoulder_roll_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.SHOULDER_LEFT_ROLL_CH].angle = angle
            self.get_logger().info(f'Set left shoulder roll (ch {self.SHOULDER_LEFT_ROLL_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir roll do ombro esquerdo: {e}')

    def right_shoulder_pitch_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.SHOULDER_RIGHT_PITCH_CH].angle = angle
            self.get_logger().info(f'Set right shoulder pitch (ch {self.SHOULDER_RIGHT_PITCH_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir pitch do ombro direito: {e}')

    def right_shoulder_yaw_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.SHOULDER_RIGHT_YAW_CH].angle = angle
            self.get_logger().info(f'Set right shoulder yaw (ch {self.SHOULDER_RIGHT_YAW_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir yaw do ombro direito: {e}')

    def right_shoulder_roll_cb(self, msg: Int32):
        angle = self.clamp_angle(msg.data)
        try:
            self.kit.servo[self.SHOULDER_RIGHT_ROLL_CH].angle = angle
            self.get_logger().info(f'Set right shoulder roll (ch {self.SHOULDER_RIGHT_ROLL_CH}) = {angle}')
        except Exception as e:
            self.get_logger().error(f'Erro ao definir roll do ombro direito: {e}')


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