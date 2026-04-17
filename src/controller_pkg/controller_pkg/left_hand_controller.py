#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

class HandControllerLeft(Node):
    def __init__(self):
        super().__init__("hand_controller_left")
        self.get_logger().info("Node [hand_controller_left] inicializado.")

        # Parâmetros (padrões: USB1 e ID 9)
        self.declare_parameter('devicename', '/dev/ttyUSB1')
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_id', 9)

        self.DEVICENAME = self.get_parameter('devicename').get_parameter_value().string_value
        self.PROTOCOL_VERSION = self.get_parameter('protocol_version').get_parameter_value().double_value
        self.BAUDRATE = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.DXL_ID = self.get_parameter('dxl_id').get_parameter_value().integer_value

        # Endereços (Protocol 2.0)
        self.ADDR_TORQUE_ENABLE    = 64
        self.ADDR_GOAL_POSITION    = 116
        self.ADDR_PRESENT_POSITION = 132

        # Limites
        self.DXL_MINIMUM_POSITION_VALUE = 0
        self.DXL_MAXIMUM_POSITION_VALUE = 4095

        # Movimento
        self.STEP_SIZE = 20
        self.TOTAL_MOVEMENT = 1900
        self.DEFAULT_DELAY = 0.01

        # Dynamixel I/O
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.port_opened = False
        self.current_position = 0
        self.previous_hand_state = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Assina mão esquerda
        self.left_hand_sub = self.create_subscription(
            Bool, "/left/hand_open", self.left_hand_callback, qos_profile
        )

        # Setup serial e torque
        self.setup_serial()
        self.set_torque(1)
        self.current_position = self.read_position()
        if self.current_position is None:
            self.get_logger().error("Não foi possível ler a posição inicial.")

    # ---- Serial ----
    def setup_serial(self):
        try:
            if self.portHandler.openPort():
                self.get_logger().info(f"Porta {self.DEVICENAME} aberta")
                self.port_opened = True
            else:
                self.get_logger().error(f"Falha ao abrir {self.DEVICENAME}")
                return False

            if self.portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().info(f"Baudrate = {self.BAUDRATE}")
                return True
            else:
                self.get_logger().error("Falha ao configurar baudrate")
                self.portHandler.closePort()
                self.port_opened = False
                return False
        except Exception as e:
            self.get_logger().error(f"Erro na configuração serial: {e}")
            if self.port_opened:
                self.portHandler.closePort()
                self.port_opened = False
            return False

    # ---- Torque ----
    def set_torque(self, enabled: int):
        if not self.port_opened:
            self.get_logger().error("Porta serial não está aberta")
            return False
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, int(enabled)
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm err: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        if dxl_error != 0:
            self.get_logger().error(f"DXL err: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False
        self.get_logger().info(f"Torque {'habilitado' if enabled else 'desabilitado'}")
        return True

    # ---- Read / Write ----
    def read_position(self):
        if not self.port_opened:
            self.get_logger().error("Porta serial não está aberta")
            return None
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm err: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None
        if dxl_error != 0:
            self.get_logger().error(f"DXL err: {self.packetHandler.getRxPacketError(dxl_error)}")
            return None
        return dxl_present_position

    def write_position(self, position: int):
        if not self.port_opened:
            self.get_logger().error("Porta serial não está aberta")
            return False
        position = max(self.DXL_MINIMUM_POSITION_VALUE, min(position, self.DXL_MAXIMUM_POSITION_VALUE))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, int(position)
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm err: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        if dxl_error != 0:
            self.get_logger().error(f"DXL err: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False
        return True

    # ---- Movimento suave ----
    def smooth_move(self, target_position: int, current_position: int):
        current_delay = self.DEFAULT_DELAY
        direction = 1 if target_position > current_position else -1
        steps = abs(target_position - current_position) // self.STEP_SIZE

        if steps == 0:
            if self.write_position(target_position):
                time.sleep(current_delay * 2)
            return target_position

        for step in range(steps):
            intermediate_position = current_position + (direction * self.STEP_SIZE * (step + 1))
            if (direction > 0 and intermediate_position > target_position) or (direction < 0 and intermediate_position < target_position):
                intermediate_position = target_position
            if self.write_position(intermediate_position):
                time.sleep(current_delay)

        if self.write_position(target_position):
            time.sleep(current_delay)
        return target_position

    # ---- Callback mão ESQUERDA ----
    def left_hand_callback(self, hand_state: Bool):
        if hand_state.data is None:
            self.get_logger().warn("Mão esquerda em estado indefinido.")
            return

        current_hand_state = bool(hand_state.data)  # True = ABERTA, False = FECHADA
        hand_state_changed = (current_hand_state != self.previous_hand_state)
        self.previous_hand_state = current_hand_state

        current_pos = self.read_position()
        if current_pos is None:
            self.get_logger().error("Não foi possível ler a posição atual do motor")
            return

        movement_made = False
        target_position = current_pos

        if hand_state_changed:
            if current_hand_state:
                # Abrir (aumenta posição)
                target_position = current_pos + self.TOTAL_MOVEMENT
                self.get_logger().info("Mão esquerda ABERTA → abrindo")
                movement_made = True
            else:
                # Fechar (diminui posição)
                target_position = current_pos - self.TOTAL_MOVEMENT
                self.get_logger().info("Mão esquerda FECHADA → fechando")
                movement_made = True

        if movement_made and target_position != current_pos:
            final_position = self.smooth_move(target_position, current_pos)
            actual_pos = self.read_position()
            if actual_pos is not None:
                self.current_position = actual_pos
                self.get_logger().info(f"Posição final: {actual_pos}")
        else:
            state_text = "aberta" if current_hand_state else "fechada"
            self.get_logger().info(f"Estado atual — mão esquerda: {state_text} (sem mudança)")

    # ---- Encerramento ----
    def cleanup_motor(self):
        if self.port_opened:
            self.set_torque(0)
            self.portHandler.closePort()
            self.port_opened = False
            self.get_logger().info("Porta fechada e torque desabilitado.")

def main(args=None):
    rclpy.init(args=args)
    node = HandControllerLeft()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrompido pelo usuário")
    finally:
        node.cleanup_motor()
        rclpy.shutdown()

if __name__ == "__main__":
    main()