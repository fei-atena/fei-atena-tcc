#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD


class HandController(Node):
    def __init__(self):
        super().__init__("hand_controller")
        self.get_logger().info("Node [hand_controller] inicializado com sucesso.")

        # Declaração dos parâmetros do ROS
        self.declare_parameter('devicename', '/dev/ttyUSB0')  # Ex: /dev/ttyUSB0, /dev/ttyACM0
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_id', 9)

        # Obtenção dos parâmetros
        self.DEVICENAME = self.get_parameter('devicename').get_parameter_value().string_value
        self.PROTOCOL_VERSION = self.get_parameter('protocol_version').get_parameter_value().double_value
        self.BAUDRATE = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.DXL_ID = self.get_parameter('dxl_id').get_parameter_value().integer_value

        # Endereços de controle
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132

        # Limites de posição
        self.DXL_MINIMUM_POSITION_VALUE = 0
        self.DXL_MAXIMUM_POSITION_VALUE = 4095

        # Configurações de movimento
        self.STEP_SIZE = 20
        self.TOTAL_MOVEMENT = 1900
        self.DEFAULT_DELAY = 0.01

        # Inicialização dos handlers
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.port_opened = False
        self.current_position = 0
        
        # Controle de estado anterior para detectar mudanças
        self.previous_hand_state = None
        
        # Configuração QoS (Quality of Service)
        # reliability: BEST_EFFORT para minimizar latência
        # durability: VOLATILE para não armazenar mensagens antigas
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # garante entrega (é estado, não sensor)
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # guarda o último valor (latched)
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.felt_hand_sub = self.create_subscription(
            Bool, 
            "/left/hand_joint", 
            self.left_hand_callback, 
            qos_profile
        )

        ## Configura a comunicação serial
        self.setup_serial()
        # Habilita o torque
        self.set_torque(1)
        # Lê a posição inicial
        self.current_position = self.read_position()
        if self.current_position is None:
            self.get_logger().error("Não foi possível ler a posição inicial do motor")

    # Função para configurar a porta serial
    def setup_serial(self):
        try:
            # Abre a porta serial
            if self.portHandler.openPort():
                self.get_logger().info("Porta serial aberta com sucesso")
                self.port_opened = True
            else:
                self.get_logger().error("Falha ao abrir a porta serial")
                return False
            
            # Configura a velocidade de transmissão
            if self.portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().info(f"Baudrate configurado para {self.BAUDRATE}")
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

    # Função para habilitar/desabilitar torque
    def set_torque(self, enabled):
        if not self.port_opened:
            self.get_logger().error("Porta serial não está aberta")
            return False
        
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, enabled
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Erro de comunicação: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            self.get_logger().error(f"Erro do Dynamixel: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False
        else:
            status = "habilitado" if enabled else "desabilitado"
            self.get_logger().info(f"Torque {status}")
            return True

    # Função para ler posição atual
    def read_position(self):
        if not self.port_opened:
            self.get_logger().error("Porta serial não está aberta")
            return None
        
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Erro de comunicação: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None
        elif dxl_error != 0:
            self.get_logger().error(f"Erro do Dynamixel: {self.packetHandler.getRxPacketError(dxl_error)}")
            return None
        return dxl_present_position

    # Função para escrever posição desejada
    def write_position(self, position):
        if not self.port_opened:
            self.get_logger().error("Porta serial não está aberta")
            return False
        
        # Limita a posição dentro dos limites
        position = max(self.DXL_MINIMUM_POSITION_VALUE, min(position, self.DXL_MAXIMUM_POSITION_VALUE))
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, position
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Erro de comunicação: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            self.get_logger().error(f"Erro do Dynamixel: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False
        return True
    
    # Função para movimento suave e lento
    def smooth_move(self, target_position, current_position):
        current_delay = self.DEFAULT_DELAY

        self.get_logger().info(f"Movendo suavemente para: {target_position}")
        self.get_logger().info(f"Velocidade: delay de {current_delay:.3f}s entre passos")

        direction = 1 if target_position > current_position else -1
        steps = abs(target_position - current_position) // self.STEP_SIZE

        if steps == 0:
            # Se a distância for menor que STEP_SIZE, move diretamente
            if self.write_position(target_position):
                time.sleep(current_delay * 2)
            return target_position

        for step in range(steps):
            intermediate_position = current_position + (direction * self.STEP_SIZE * (step + 1))

            # Garante que não ultrapasse o alvo
            if (direction > 0 and intermediate_position > target_position) or (direction < 0 and intermediate_position < target_position):
                intermediate_position = target_position

            if self.write_position(intermediate_position):
                time.sleep(current_delay)

        # Movimento final para garantir posição exata
        if self.write_position(target_position):
            time.sleep(current_delay)

        return target_position

    
    # Callback para o estado da mão esquerda
    # Roda quando uma nova mensagem é recebida no tópico /left/hand_joint
    # TODO: depois de realizar a beta remover logger.info para uma aplicação mais limpa e leve
    def left_hand_callback(self, hand_state: Bool):
        if hand_state.data is None:
            self.get_logger().warn("Mão esquerda está em estado indefinido.")
            return

        current_hand_state = hand_state.data
        
        # Verifica se houve mudança de estado
        hand_state_changed = (current_hand_state != self.previous_hand_state)
        
        # Atualiza o estado anterior
        self.previous_hand_state = current_hand_state
        
        # Lê posição atual do motor
        current_pos = self.read_position()
        if current_pos is None:
            self.get_logger().error("Não foi possível ler a posição atual do motor")
            return
        
        # Processa movimento apenas se houve mudança de estado
        movement_made = False
        target_position = current_pos
        
        if hand_state_changed:
            if current_hand_state:
                # Mão fechada - move o motor numa direção
                target_position = current_pos - self.TOTAL_MOVEMENT
                self.get_logger().info("Mão esquerda FECHADA detectada - movendo motor")
                movement_made = True
            else:
                # Mão aberta - move o motor na direção oposta  
                target_position = current_pos + self.TOTAL_MOVEMENT
                self.get_logger().info("Mão esquerda ABERTA detectada - movendo motor")
                movement_made = True
        
        # Se houve movimento e a posição mudou, move o motor suavemente
        if movement_made and target_position != current_pos:
            self.get_logger().info(f"Movimento iniciado de {current_pos} para {target_position}")
            
            # Movimento suave e lento
            final_position = self.smooth_move(target_position, current_pos)
            
            # Lê a posição atual para verificar
            actual_pos = self.read_position()
            if actual_pos is not None:
                self.get_logger().info(f"Posição final do motor: {actual_pos}")
                self.current_position = actual_pos
        else:
            # Apenas exibe o estado atual sem mover o motor
            state_text = "fechada" if current_hand_state else "aberta"
            self.get_logger().info(f"Estado atual - Mão esquerda: {state_text} (sem mudança)")

    # Desabilita o torque e fecha a comunicação serial
    def cleanup_motor(self):
        if self.port_opened:
            self.get_logger().info("Desabilitando torque e fechando comunicação serial...")
            self.set_torque(0)
            self.portHandler.closePort()
            self.port_opened = False
            self.get_logger().info("Motor desconectado com sucesso")



def main(args=None):
    rclpy.init(args=args)
    node = HandController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Programa interrompido pelo usuário")
    finally:
        # Limpeza do motor antes de sair
        node.cleanup_motor()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

