#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32

# PCA9685
from adafruit_servokit import ServoKit

# ====== Frequência de atualização ======
FLUSH_RATE_HZ = 20.0  # 20 Hz, atualiza SEMPRE

# ====== Mapeamento de canais (ajuste conforme sua fiação) ======
# left:  ch0=abd, ch1=flex, ch2=rot, ch3=elbow
# right: ch4=abd, ch5=flex, ch6=rot, ch7=elbow
CH_LEFT_SH_ABD   = 0
CH_LEFT_SH_FLEX  = 1
CH_LEFT_SH_ROT   = 2
CH_LEFT_ELBOW    = 3

CH_RIGHT_SH_ABD  = 4
CH_RIGHT_SH_FLEX = 5
CH_RIGHT_SH_ROT  = 6
CH_RIGHT_ELBOW   = 7

# ====== OFFSETS (graus, inteiros) ======
# Ajuste fino de zero mecânico. Ex.: +3 deixa o servo 3° mais “aberto”.
OFFSETS = {
    'left_sh_abd':   0,
    'left_sh_flex':  20,
    'left_sh_rot':   0,
    'left_elbow':    0,
    'right_sh_abd':  10,
    'right_sh_flex': 0,
    'right_sh_rot':  0,
    'right_elbow':   0,
}

class JointsPCAControl(Node):
    def __init__(self):
        super().__init__('joints_pca_control')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- ServoKit / PCA9685 ----
        self.kit = ServoKit(channels=16)
        # Faixa de pulso padrão "hobby" (500–2500 µs) para os 8 canais usados
        for ch in (CH_LEFT_SH_ABD, CH_LEFT_SH_FLEX, CH_LEFT_SH_ROT, CH_LEFT_ELBOW,
                   CH_RIGHT_SH_ABD, CH_RIGHT_SH_FLEX, CH_RIGHT_SH_ROT, CH_RIGHT_ELBOW):
            try:
                self.kit.servo[ch].set_pulse_width_range(500, 2500)
            except Exception as e:
                self.get_logger().warning(f'PulseRange falhou no ch{ch}: {e}')

        # ---- Últimos comandos (graus, inteiros) ----
        self.left_sh_abd   = 0
        self.left_sh_flex  = 0
        self.left_sh_rot   = 0
        self.left_elbow    = 0
        self.right_sh_abd  = 0
        self.right_sh_flex = 0
        self.right_sh_rot  = 0
        self.right_elbow   = 0

        # ---- Subscriptions: apenas guardam os valores recebidos ----
        # ESQUERDO
        self.create_subscription(Int32, '/left/shoulder_abd',   lambda m: self._set_left('abd',  m.data), qos)
        self.create_subscription(Int32, '/left/shoulder_flex',  lambda m: self._set_left('flex', m.data), qos)
        self.create_subscription(Int32, '/left/shoulder_rot',   lambda m: self._set_left('rot',  m.data), qos)
        self.create_subscription(Int32, '/left/elbow_flex',     lambda m: self._set_left('elbow',m.data), qos)
        # DIREITO
        self.create_subscription(Int32, '/right/shoulder_abd',  lambda m: self._set_right('abd',  m.data), qos)
        self.create_subscription(Int32, '/right/shoulder_flex', lambda m: self._set_right('flex', m.data), qos)
        self.create_subscription(Int32, '/right/shoulder_rot',  lambda m: self._set_right('rot',  m.data), qos)
        self.create_subscription(Int32, '/right/elbow_flex',    lambda m: self._set_right('elbow',m.data), qos)

        # ---- Timer: escreve TODOS os canais SEMPRE (sem for) ----
        self.create_timer(1.0/FLUSH_RATE_HZ, self._flush)

        self.get_logger().info('PCA control: offsets inteiros + flush síncrono 20 Hz (sem loop nem clamp).')

    # ===== setters =====
    def _as_int(self, v) -> int:
        try:
            return int(v)
        except Exception:
            return 0

    def _set_left(self, which: str, val):
        a = self._as_int(val)
        if   which == 'abd':   self.left_sh_abd  = a
        elif which == 'flex':  self.left_sh_flex = a
        elif which == 'rot':   self.left_sh_rot  = a
        elif which == 'elbow': self.left_elbow   = a

    def _set_right(self, which: str, val):
        a = self._as_int(val)
        if   which == 'abd':   self.right_sh_abd  = a
        elif which == 'flex':  self.right_sh_flex = a
        elif which == 'rot':   self.right_sh_rot  = a
        elif which == 'elbow': self.right_elbow   = a

    # ===== flush: aplica offsets e envia ao PCA =====
    def _flush(self):
        # LEFT
        self.kit.servo[CH_LEFT_SH_ABD].angle  = self.left_sh_abd  + OFFSETS['left_sh_abd']
        self.kit.servo[CH_LEFT_SH_FLEX].angle = self.left_sh_flex + OFFSETS['left_sh_flex']
        self.kit.servo[CH_LEFT_SH_ROT].angle  = self.left_sh_rot  + OFFSETS['left_sh_rot']
        self.kit.servo[CH_LEFT_ELBOW].angle   = self.left_elbow   + OFFSETS['left_elbow']
        # RIGHT
        self.kit.servo[CH_RIGHT_SH_ABD].angle  = self.right_sh_abd  + OFFSETS['right_sh_abd']
        self.kit.servo[CH_RIGHT_SH_FLEX].angle = self.right_sh_flex + OFFSETS['right_sh_flex']
        self.kit.servo[CH_RIGHT_SH_ROT].angle  = self.right_sh_rot  + OFFSETS['right_sh_rot']
        self.kit.servo[CH_RIGHT_ELBOW].angle   = self.right_elbow   + OFFSETS['right_elbow']


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