#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Int32
import json
from tf_transformations import euler_from_quaternion

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
                'pitch':  {'in_min': -90.0, 'in_max': 30.0,  'out_min': 0,   'out_max': 110, 'invert': True},
                'roll':   {'in_min': -180.0, 'in_max': 180.0, 'out_min': 0,   'out_max': 180, 'invert': False}
            },
            'ElbowLeft': {
                'pitch': {
                    'in_min': -90.0, 'in_max': 0.0, 'out_min': 0, 'out_max': 80, 'invert': False
                }
            },
            'ShoulderRight': {
                'yaw':    {'in_min': -90.0, 'in_max': 180.0, 'out_min': 0,   'out_max': 90,  'invert': False},
                'pitch':  {'in_min': -90.0, 'in_max': 30.0,  'out_min': 0,   'out_max': 110, 'invert': True},
                'roll':   {'in_min': -180.0, 'in_max': 180.0, 'out_min': 0,   'out_max': 180, 'invert': False}
            },
            'ElbowRight': {
                'pitch': {
                    'in_min': -90.0, 'in_max': 0.0, 'out_min': 0, 'out_max': 100, 'invert': False
                }
            }
        }

        # Centralized configuration for servo motor limits
        self.servo_limits = {
            'ShoulderLeft': {
                'roll': {'min': 0, 'max': 180},
                'pitch': {'min': 0, 'max': 110},
                'yaw': {'min': 0, 'max': 90}
            },
            'ElbowLeft': {
                'pitch': {'min': 0, 'max': 80}
            },
            'ShoulderRight': {
                'roll': {'min': 0, 'max': 180},
                'pitch': {'min': 0, 'max': 110},
                'yaw': {'min': 0, 'max': 90}
            },
            'ElbowRight': {
                'pitch': {'min': 0, 'max': 100}
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

    def quaternion_to_euler(self, ox, oy, oz, ow):
        """Converte quaternion para ângulos de Euler (roll, pitch, yaw) em graus normalizados (0-360)."""
        try:
            orientation_list = [ox, oy, oz, ow]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            # Convertendo de radianos para graus
            roll = roll * (180.0 / 3.141592653589793)
            pitch = pitch * (180.0 / 3.141592653589793)
            yaw = yaw * (180.0 / 3.141592653589793)

            # Normalizando para o intervalo 0-360 graus
            roll = (roll + 360) % 360
            pitch = (pitch + 360) % 360
            yaw = (yaw + 360) % 360

            self.get_logger().info(f'Quaternion convertido: ox={ox}, oy={oy}, oz={oz}, ow={ow} -> roll={roll}, pitch={pitch}, yaw={yaw}')
            return roll, pitch, yaw
        except Exception as e:
            self.get_logger().error(f'Erro ao converter quaternion para Euler: {str(e)}')
            return 0.0, 0.0, 0.0

    def convert_all_quaternions_to_euler(self, orientations):
        """Converte todos os quaternions para ângulos de Euler e retorna um dicionário."""
        euler_data = {}
        for joint_name, quaternion in orientations.items():
            ox, oy, oz, ow = quaternion['ox'], quaternion['oy'], quaternion['oz'], quaternion['ow']
            roll, pitch, yaw = self.quaternion_to_euler(ox, oy, oz, ow)
            euler_data[joint_name] = {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }
        return euler_data

    def save_complete_euler_payload(self, euler_data):
        """Salva o payload completo de Euler em um arquivo JSON."""
        try:
            with open('/home/atena/fei-atena-tcc/euler.json', 'w') as file:
                json.dump(euler_data, file, indent=4)
        except Exception as e:
            self.get_logger().error(f'Erro ao salvar euler.json: {str(e)}')

    def apply_servo_limits(self, joint_name, angle_type, value):
        """Aplica os limites definidos para os servos e retorna o valor ajustado."""
        if joint_name in self.servo_limits and angle_type in self.servo_limits[joint_name]:
            limits = self.servo_limits[joint_name][angle_type]
            limited_value = max(limits['min'], min(limits['max'], value))
            self.get_logger().info(
                f'Aplicando limites: joint={joint_name}, angle_type={angle_type}, valor_original={value}, limitado={limited_value}')
            return limited_value
        else:
            self.get_logger().warning(
                f'Limites não definidos para joint={joint_name}, angle_type={angle_type}. Usando valor original: {value}')
            return value

    def process_joint_angles(self, joint_name, angles):
        """Processa os ângulos das juntas, aplica limites e mapeia para os valores dos servos."""
        servo_values = {}
        for angle_type, value in angles.items():
            limited_value = self.apply_servo_limits(joint_name, angle_type, value)
            if joint_name in self.joint_mappings and angle_type in self.joint_mappings[joint_name]:
                cfg = self.joint_mappings[joint_name][angle_type]
                mapped_value = self.map_angle_to_servo(
                    limited_value,
                    cfg['in_min'], cfg['in_max'],
                    cfg['out_min'], cfg['out_max'],
                    cfg.get('invert', False)
                )
                self.get_logger().info(
                    f'Mapeado para servo: joint={joint_name}, angle_type={angle_type}, entrada={limited_value}, mapeado={mapped_value}')
                servo_values[angle_type] = mapped_value
            else:
                servo_values[angle_type] = self.normalize_angle(limited_value)
                self.get_logger().info(
                    f'Fallback mapping: joint={joint_name}, angle_type={angle_type}, entrada={limited_value}, mapeado={servo_values[angle_type]}')
        return servo_values

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
                if 'local_orientation_quat' in body_data:
                    orientations = body_data['local_orientation_quat']
                    euler_data = self.convert_all_quaternions_to_euler(orientations)
                    self.save_complete_euler_payload(euler_data)

                    for joint_name, angles in euler_data.items():
                        if joint_name in self.joint_publishers:
                            servo_values = self.process_joint_angles(joint_name, angles)
                            for angle_type, mapped_value in servo_values.items():
                                if angle_type in self.joint_publishers[joint_name]:
                                    msg = Int32()
                                    msg.data = mapped_value
                                    self.joint_publishers[joint_name][angle_type].publish(msg)
                                    self.get_logger().info(
                                        f'Publicado: joint={joint_name}, angle_type={angle_type}, valor_servo={mapped_value}')
        except Exception as e:
            self.get_logger().error(f'Erro ao processar dados: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()