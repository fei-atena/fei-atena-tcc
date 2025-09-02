#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Usaremos Float64 para os valores com casas decimais
import json # Importa a biblioteca para trabalhar com JSON

class ProcessamentoNode(Node):
    def __init__(self):
        super().__init__('processamento_node')

        # 1. Armazene o JSON que você vai receber cSSomo uma string.
        #    Na prática, você receberia isso de um socket, arquivo, etc.
        self.json_data_string = """
        {
            "finger_distances": {
                "LeftHand": {
                    "Thumb": 56.97,
                    "Index": 100.0,
                    "Ring": 63.33,
                    "Pinky": 59.47,
                    "Middle": 64.61
                },
                "RightHand": {
                    "Thumb": 53.92,
                    "Middle": 36.16,
                    "Ring": 29.07,
                    "Pinky": 28.08,
                    "Index": 34.93
                }
            }
        }
        """

        # 2. Crie um dicionário para guardar todos os seus publishers
        self.publishers_ = {}
        
        # Converte o JSON em um dicionário Python para configurar os tópicos
        dados = json.loads(self.json_data_string)
        
        # 3. Itere sobre os dados para criar um publisher para cada dedo
        for hand, fingers in dados['finger_distances'].items():
            for finger, _ in fingers.items():
                # Cria um nome de tópico dinâmico, ex: /finger_distances/lefthand/thumb
                topic_name = f'finger_distances/{hand.lower()}/{finger.lower()}'
                
                # Cria o publisher e o armazena no dicionário
                self.publishers_[topic_name] = self.create_publisher(Float64, topic_name, 10)
                self.get_logger().info(f'Tópico criado: "{topic_name}"')

        # 4. Crie um timer para chamar a função de publicação periodicamente
        timer_period = 1.0  # segundos
        self.timer = self.create_timer(timer_period, self.publish_distances)
        self.get_logger().info('Processamento Node iniciado. Publicando distâncias dos dedos...')

    def publish_distances(self):
        """
        Esta função é chamada pelo timer. Ela lê o JSON,
        e publica cada valor no seu tópico correspondente.
        """
        # Converte a string JSON para um dicionário Python
        dados = json.loads(self.json_data_string)

        # Itera pelos dados e publica cada um
        for hand, fingers in dados['finger_distances'].items():
            for finger, distance in fingers.items():
                topic_name = f'finger_distances/{hand.lower()}/{finger.lower()}'
                
                # Prepara a mensagem do tipo Float64
                msg = Float64()
                msg.data = float(distance) # Garante que o dado é um float
                
                # Publica a mensagem no tópico correto
                self.publishers_[topic_name].publish(msg)
                
                # Log para sabermos o que está sendo publicado
                self.get_logger().info(f'Publicando em "{topic_name}": {msg.data}')
        
        self.get_logger().info('--- Ciclo de publicação completo ---')


def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()