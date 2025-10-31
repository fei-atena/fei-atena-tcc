#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Int32
import json, os
from math import pi
from tf_transformations import (
    euler_from_quaternion,
    quaternion_multiply,
    quaternion_inverse,
)

DEG = 180.0 / pi

def clamp(v, a, b):
    return max(a, min(b, v))

def q_mul(q1, q2):  # (x,y,z,w)
    return quaternion_multiply(q1, q2)

def q_inv(q):
    return quaternion_inverse(q)

def euler_deg(q, axes='sxyz'):
    r, p, y = euler_from_quaternion(q, axes=axes)
    return (r*DEG, p*DEG, y*DEG)

def build_quat(d):  # dict -> (x,y,z,w)
    return [d['ox'], d['oy'], d['oz'], d['ow']]

class ProcessamentoNode(Node):
    def __init__(self):
        super().__init__('processamento_node')
        self.get_logger().info('Processamento ZED → Ângulos anatômicos (flex/abd/rot)...')

        # Arquivos
        self.json_file_path  = '/home/atena/fei-atena-tcc/recebido.json'
        self.calib_file_path = '/home/atena/fei-atena-tcc/calib.json'

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers de mão (abre/fecha)
        self.pub_hand = {
            'left':  self.create_publisher(Bool,  '/left/hand_open',  qos),
            'right': self.create_publisher(Bool,  '/right/hand_open', qos),
        }

        # Publishers anatômicos (ombro 3DoF + cotovelo flex)
        self.pub = {
            'left': {
                'shoulder_flex': self.create_publisher(Int32, '/left/shoulder_flex', qos),
                'shoulder_abd':  self.create_publisher(Int32, '/left/shoulder_abd',  qos),
                'shoulder_rot':  self.create_publisher(Int32, '/left/shoulder_rot',  qos),
                'elbow_flex':    self.create_publisher(Int32, '/left/elbow_flex',    qos),
            },
            'right': {
                'shoulder_flex': self.create_publisher(Int32, '/right/shoulder_flex', qos),
                'shoulder_abd':  self.create_publisher(Int32, '/right/shoulder_abd',  qos),
                'shoulder_rot':  self.create_publisher(Int32, '/right/shoulder_rot',  qos),
                'elbow_flex':    self.create_publisher(Int32, '/right/elbow_flex',    qos),
            }
        }

        # Limites físicos (graus no servo)
        self.servo_limits = {'min': 0, 'max': 140}

        # Suavização (0=sem filtro, 0.2 recomendado)
        self.alpha = 0.2
        self.prev = {}

        # Carrega calibração (alinhamento + neutro)
        self.calib = self.load_calib()

        # Mapeamentos (por lado)
        # - euler_axes: ordem de extração
        # - axis_map: qual eixo do Euler vira cada ação
        # - theta_min/max: faixa esperada do ângulo humano
        # - invert: espelha sentido
        # - offset: correção fina (graus) pós-Euler
        self.mapping = {
            'left': {
                'Shoulder': {
                    'euler_axes': 'sxyz',
                    'axis_map': {'flex': 'x', 'abd': 'y', 'rot': 'z'},
                    'ranges': {
                        'flex': {'theta_min': -90, 'theta_max':  90, 'invert': False, 'offset': 0},
                        'abd':  {'theta_min': -10, 'theta_max': 120, 'invert': False, 'offset': 0},
                        'rot':  {'theta_min': -90, 'theta_max':  90, 'invert': False, 'offset': 0},
                    }
                },
                'Elbow': {
                    'relative': True,                  # antebraço relativo ao úmero
                    'euler_axes': 'sxyz',
                    'axis_map': {'flex': 'x'},
                    'ranges': {
                        'flex': {'theta_min': 0, 'theta_max': 140, 'invert': True, 'offset': 0}
                    }
                }
            },
            'right': {
                'Shoulder': {
                    'euler_axes': 'sxyz',
                    'axis_map': {'flex': 'x', 'abd': 'y', 'rot': 'z'},
                    'ranges': {
                        'flex': {'theta_min': -90, 'theta_max':  90, 'invert': False,  'offset': 0},
                        'abd':  {'theta_min': -10, 'theta_max': 120, 'invert': True,  'offset': 0},
                        'rot':  {'theta_min': -90, 'theta_max':  90, 'invert': True,  'offset': 0},
                    }
                },
                'Elbow': {
                    'relative': True,
                    'euler_axes': 'sxyz',
                    'axis_map': {'flex': 'x'},
                    'ranges': {
                        'flex': {'theta_min': 0, 'theta_max': 140, 'invert': False, 'offset': 0}
                    }
                }
            }
        }

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    # ---------------- CALIB ----------------
    def load_calib(self):
        """
        Estrutura:
        {
          "left": {
            "Shoulder": {"neutral": [0,0,0,1], "align": [0,0,0,1]},
            "Elbow":    {"neutral": [0,0,0,1], "align": [0,0,0,1]}
          },
          "right": { ... }
        }
        """
        default = {
            'left':  {'Shoulder': {'neutral':[0,0,0,1], 'align':[0,0,0,1]},
                      'Elbow':    {'neutral':[0,0,0,1], 'align':[0,0,0,1]}},
            'right': {'Shoulder': {'neutral':[0,0,0,1], 'align':[0,0,0,1]},
                      'Elbow':    {'neutral':[0,0,0,1], 'align':[0,0,0,1]}}
        }
        try:
            if os.path.exists(self.calib_file_path):
                with open(self.calib_file_path,'r') as f:
                    data = json.load(f)
                    # validação simples
                    for side in ['left','right']:
                        for j in ['Shoulder','Elbow']:
                            data.setdefault(side,{}).setdefault(j,{})
                            data[side][j].setdefault('neutral',[0,0,0,1])
                            data[side][j].setdefault('align',[0,0,0,1])
                    return data
        except Exception as e:
            self.get_logger().warn(f'Falha ao ler calib.json: {e}')
        return default

    # ---------------- UTIL ----------------
    def smooth(self, key, value):
        if self.alpha <= 0: return value
        prev = self.prev.get(key, value)
        y = (1.0 - self.alpha) * value + self.alpha * prev
        self.prev[key] = y
        return y

    def map_servo(self, theta, r):  # theta em graus
        # offset pós-Euler
        theta_corr = theta + r.get('offset', 0.0)
        if r.get('invert', False):
            theta_corr = -theta_corr
        tmin = r['theta_min']; tmax = r['theta_max']
        omin = self.servo_limits['min']; omax = self.servo_limits['max']
        if tmax == tmin:
            s = (omin + omax) * 0.5
        else:
            s = omin + (theta_corr - tmin) * (omax - omin) / (tmax - tmin)
        return int(clamp(round(s), omin, omax))

    # ---------------- CORE ----------------
    def loop(self):
        dados = self.read_json_safely()
        if not dados: return

        # mãos
        hands = dados.get('current_frame',{}).get('hands',{})
        self.pub_hand['left'].publish(Bool(data=hands.get('left_hand_open', False)))
        self.pub_hand['right'].publish(Bool(data=hands.get('right_hand_open', False)))

        body_list = dados.get('current_frame',{}).get('body',{}).get('body_list',[])
        if not body_list: return
        body = body_list[0]
        quats = body.get('local_orientation_quat',{})
        # Esperados (ZED): ShoulderLeft, ElbowLeft, ShoulderRight, ElbowRight
        req = ['ShoulderLeft','ElbowLeft','ShoulderRight','ElbowRight']
        if not all(k in quats for k in req): return

        # Braco ESQUERDO
        self.process_side('left',
                          shoulder_q=build_quat(quats['ShoulderLeft']),
                          elbow_q=build_quat(quats['ElbowLeft']))
        # Braco DIREITO
        self.process_side('right',
                          shoulder_q=build_quat(quats['ShoulderRight']),
                          elbow_q=build_quat(quats['ElbowRight']))

    def process_side(self, side, shoulder_q, elbow_q):
        m = self.mapping[side]

        # Ombro relativo (tronco→úmero), com alinhamento e neutral
        q_align_s   = self.calib[side]['Shoulder']['align']
        q_neutral_s = self.calib[side]['Shoulder']['neutral']
        q_rel_s = q_mul(q_align_s, q_mul(shoulder_q, q_inv(q_neutral_s)))

        ex, ey, ez = euler_deg(q_rel_s, axes=m['Shoulder']['euler_axes'])
        axes_map = {'x': ex, 'y': ey, 'z': ez}

        # Publica flex/abd/rot do ombro
        for action in ['flex','abd','rot']:
            axis = m['Shoulder']['axis_map'][action]
            theta = axes_map[axis]
            rng = m['Shoulder']['ranges'][action]
            val = self.map_servo(theta, rng)
            key = f'{side}.shoulder_{action}'
            val = int(round(self.smooth(key, val)))
            self.pub[side][f'shoulder_{action}'].publish(Int32(data=val))
            self.get_logger().info(f'{key}: θ={theta:.1f}° → servo={val}')

        # Cotovelo relativo (úmero→antebraço)
        q_align_e   = self.calib[side]['Elbow']['align']
        q_neutral_e = self.calib[side]['Elbow']['neutral']
        # rotação antebraço no frame do úmero
        q_raw_ef = q_mul(q_inv(shoulder_q), elbow_q)
        q_rel_e  = q_mul(q_align_e, q_mul(q_raw_ef, q_inv(q_neutral_e)))

        ex, ey, ez = euler_deg(q_rel_e, axes=m['Elbow']['euler_axes'])
        axes_map_e = {'x': ex, 'y': ey, 'z': ez}

        # Publica flexão do cotovelo
        rng_e = m['Elbow']['ranges']['flex']
        theta_e = axes_map_e[m['Elbow']['axis_map']['flex']]
        val_e = self.map_servo(theta_e, rng_e)
        key = f'{side}.elbow_flex'
        val_e = int(round(self.smooth(key, val_e)))
        self.pub[side]['elbow_flex'].publish(Int32(data=val_e))
        self.get_logger().info(f'{key}: θ={theta_e:.1f}° → servo={val_e}')

    def read_json_safely(self):
        try:
            with open(self.json_file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f'Erro ao ler JSON: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
