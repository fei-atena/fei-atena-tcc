#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import json

class ProcessamentoNode(Node):
    def __init__(self):
        super().__init__('processamento_node')

        # Caminho do arquivo JSON
        self.json_file_path = '/home/vitor-lucas-fujita-fel-cio/Documents/recebido.json'

        # Crie os publishers para cada mão
        self.publishers_ = {
            'righthand': self.create_publisher(Bool, 'hand_status/righthand', 10),
            'lefthand': self.create_publisher(Bool, 'hand_status/lefthand', 10)
        }

        # Timer para ler o arquivo periodicamente
        timer_period = 0.1  # 100ms para leitura mais frequente
        self.timer = self.create_timer(timer_period, self.publish_hand_status)
        self.get_logger().info('Processamento Node iniciado. Monitorando status das mãos...')

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