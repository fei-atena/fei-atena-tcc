#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Int32
import json

class ProcessamentoNode(Node):
    def __init__(self):
        super().__init__('processamento_node')
        self.get_logger().info('Processamento Node iniciado. Monitorando status das mãos e ângulos...')
        
        # Caminho do arquivo JSON
        self.json_file_path = '/home/atena/fei-atena-tcc/recebido.json'

        # Configuração QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers para mãos
        self.hand_publishers = {
            'right_hand': self.create_publisher(Bool, "/right/hand_joint", qos_profile),
            'left_hand': self.create_publisher(Bool, "/left/hand_joint", qos_profile)
        }

        # Publishers para juntas
        self.joint_publishers = {
            'ShoulderLeft': {
                'pitch': self.create_publisher(Int32, "/left/shoulder_pitch", qos_profile),
                'roll': self.create_publisher(Int32, "/left/shoulder_roll", qos_profile),
                'yaw': self.create_publisher(Int32, "/left/shoulder_yaw", qos_profile)
            },
            'ElbowLeft': {
                'pitch': self.create_publisher(Int32, "/left/elbow_pitch", qos_profile)
            },
            'ShoulderRight': {
                'pitch': self.create_publisher(Int32, "/right/shoulder_pitch", qos_profile),
                'roll': self.create_publisher(Int32, "/right/shoulder_roll", qos_profile),
                'yaw': self.create_publisher(Int32, "/right/shoulder_yaw", qos_profile)
            },
            'ElbowRight': {
                'pitch': self.create_publisher(Int32, "/right/elbow_pitch", qos_profile)
            }
        }

        self.timer = self.create_timer(0.1, self.publish_status)

    def read_json_file(self):
        try:
            with open(self.json_file_path, 'r') as file:
                return json.load(file)
        except Exception as e:
            self.get_logger().error(f'Erro ao ler arquivo JSON: {str(e)}')
            return None

    def normalize_angle(self, angle):
        """Converte ângulos para a faixa 0-180"""
        # Mapeia o ângulo para 0-180
        normalized = int((angle + 180) * (180/360))
        return max(0, min(180, normalized))  # Garante que está entre 0 e 180

    def publish_status(self):
        """Lê o arquivo JSON e publica todos os status"""
        try:
            dados = self.read_json_file()
            if dados is None or 'current_frame' not in dados:
                return

            # Publica status das mãos
            if 'hands' in dados['current_frame']:
                hands = dados['current_frame']['hands']
                
                # Publica status da mão direita
                right_msg = Bool()
                right_msg.data = hands.get('right_hand_open', False)
                self.hand_publishers['right_hand'].publish(right_msg)
                self.get_logger().info(f'Publicando mão direita: {right_msg.data}')
                
                # Publica status da mão esquerda
                left_msg = Bool()
                left_msg.data = hands.get('left_hand_open', False)
                self.hand_publishers['left_hand'].publish(left_msg)
                self.get_logger().info(f'Publicando mão esquerda: {left_msg.data}')

            # Publica ângulos das juntas
            if ('body' in dados['current_frame'] and 
                'body_list' in dados['current_frame']['body'] and 
                len(dados['current_frame']['body']['body_list']) > 0):
                
                body_data = dados['current_frame']['body']['body_list'][0]
                if 'local_orientation_euler_deg' in body_data:
                    orientations = body_data['local_orientation_euler_deg']
                    
                    for joint_name, angles in orientations.items():
                        if joint_name in self.joint_publishers:
                            for angle_type, value in angles.items():
                                angle_type = angle_type.lower()
                                if angle_type in self.joint_publishers[joint_name]:
                                    msg = Int32()
                                    msg.data = self.normalize_angle(value)
                                    self.joint_publishers[joint_name][angle_type].publish(msg)
                                    self.get_logger().info(f'Publicando {joint_name} {angle_type}: {msg.data}')

        except Exception as e:
            self.get_logger().error(f'Erro ao processar dados: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()