import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher_node')
        self.get_logger().info('Hello from the main_node!')

def main(args=None):
    """
    이 노드를 실행하기 위한 메인 함수입니다.
    """
    rclpy.init(args=args)

    node = MinimalPublisher()

    # 실제로는 rclpy.spin(node)를 사용하지만,
    # CI에서 즉시 종료되도록 하기 위해 바로 destroy 합니다.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
