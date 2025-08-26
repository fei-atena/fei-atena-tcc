#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32

class ProcessamentoNode(Node):
    def __init__(self):
        super().__init__('processamento_node')
        self.publisher_ = self.create_publisher(Int32, 'test', 10)
        # Exemplo de publicação periódica
        timer_period = 0.5  # segundos
        self.timer = self.create_timer(timer_period, self.publish_num)
        self.count = 0
        self.get_logger().info('Processamento Node iniciado e publicando em "test"')

    def publish_num(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)

    def publish_message(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ProcessamentoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()