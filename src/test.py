import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__('my_publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publisher node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from the autonomous car!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
