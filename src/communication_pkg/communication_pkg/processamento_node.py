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
            # ShoulderLeft hardware: temos apenas 2 servos disponíveis
            # - servo para pitch (ex: channel 2) -> range aproximado 0..110
            # - servo para yaw   (ex: channel 0) -> range aproximado 0..90
            # Não existe um servo separado para roll neste ombro, portanto
            # removemos o publisher para 'roll' e vamos trabalhar junta por junta.
            'ShoulderLeft': {
                'pitch': self.create_publisher(Int32, "/left/shoulder_pitch", qos_profile),
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

        # Per-joint mapping configuration for converting input Euler angles
        # (degrees from the body tracker) into servo-friendly ranges.
        # Assumptions made (can be tuned):
        # - Input angles are in degrees, roughly in [-180, 180].
        # - For ShoulderLeft: yaw servo (ch0) accepts [0..90], pitch servo (ch2) accepts [0..110].
        # - Input ranges chosen to capture expected motion (see comments below); values are
        #   clamped to the servo output range.
        self.joint_mappings = {
            'ShoulderLeft': {
                'yaw':    {'in_min': -90.0, 'in_max': 180.0, 'out_min': 0,   'out_max': 90,  'invert': False},
                # pitch: forward lift in the source is negative (~-60..-80). We map input
                # roughly from [-90 .. 30] -> [0 .. 110] so large negative pitch sends servo high.
                'pitch':  {'in_min': -90.0, 'in_max': 30.0,  'out_min': 0,   'out_max': 110, 'invert': True},
                # note: no roll mapping because hardware não tem servo para roll no ombro
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

    def map_angle_to_servo(self, angle, in_min, in_max, out_min, out_max, invert=False):
        """Mapeia um ângulo de entrada (graus) para o intervalo do servo e retorna inteiro.

        - angle: valor de entrada em graus (float)
        - in_min/in_max: faixa esperada de entrada
        - out_min/out_max: faixa do servo (ex: 0..90)
        - invert: se True, inverte o mapeamento (útil quando valores negativos devem corresponder a valores maiores)
        """
        try:
            a = float(angle)
        except Exception:
            # se não for um número, retorne o valor médio do servo
            return int((out_min + out_max) / 2)

        # evita divisão por zero
        if in_max == in_min:
            t = 0.5
        else:
            t = (a - in_min) / (in_max - in_min)

        if invert:
            t = 1.0 - t

        # aplicar clamp em t e calcular saída
        t = max(0.0, min(1.0, t))
        out = out_min + t * (out_max - out_min)
        return int(round(out))

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

                                    # Se existir uma configuração de mapeamento para essa junta/ângulo,
                                    # use-a; caso contrário, caia no comportamento legacy (0-180).
                                    if (joint_name in self.joint_mappings and
                                            angle_type in self.joint_mappings[joint_name]):
                                        cfg = self.joint_mappings[joint_name][angle_type]
                                        mapped = self.map_angle_to_servo(
                                            value,
                                            cfg['in_min'], cfg['in_max'],
                                            cfg['out_min'], cfg['out_max'],
                                            cfg.get('invert', False)
                                        )
                                        msg.data = mapped
                                        self.get_logger().info(
                                            f'Mapeado {joint_name} {angle_type}: entrada={value} -> servo={msg.data}')
                                    else:
                                        # fallback genérico
                                        msg.data = self.normalize_angle(value)
                                        self.get_logger().info(f'Publicando (fallback) {joint_name} {angle_type}: {msg.data}')

                                    self.joint_publishers[joint_name][angle_type].publish(msg)

        except Exception as e:
            self.get_logger().error(f'Erro ao processar dados: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()