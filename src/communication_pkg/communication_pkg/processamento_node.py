#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
import json

class ProcessamentoNode(Node):
    def __init__(self):
        super().__init__('processamento_node')
        self.get_logger().info('Processamento Node iniciado. Monitorando status das mãos...')
        
        # Caminho do arquivo JSON
        # TODO: Atualize este caminho conforme necessário (utilize um caminho relativo ou parâmetro)
        self.json_file_path = '/home/vitor-lucas-fujita-fel-cio/Documents/recebido.json' #! troque para " "

        # Configuração QoS (Quality of Service)
        # reliability: BEST_EFFORT para minimizar latência
        # durability: VOLATILE para não armazenar mensagens antigas
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # garante entrega (é estado, não sensor)
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # guarda o último valor (latched)
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Crie os publishers para cada mão
        self.publishers_ = {
            'righthand': self.create_publisher(Bool, "/right/hand_joint", qos_profile),
            'lefthand': self.create_publisher(Bool, "/left/hand_joint", qos_profile)
        }

        self.timer = self.create_timer(0.1, self.publish_hand_status)

    def read_json_file(self):
        try:
            with open(self.json_file_path, 'r') as file:
                return json.load(file)
        except Exception as e:
            self.get_logger().error(f'Erro ao ler arquivo JSON: {str(e)}')
            return None

    def publish_hand_status(self):
        """
        Lê o arquivo JSON e publica o status de cada mão
        """
        dados = self.read_json_file()
        
        if dados is None:
            return
            
        # Publica o status de cada mão
        for hand, status in dados.items():
            msg = Bool()
            msg.data = status
            self.publishers_[hand].publish(msg)
            self.get_logger().info(f'Publicando status da {hand}: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()