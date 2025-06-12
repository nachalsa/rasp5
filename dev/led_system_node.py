import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import led_system


class LedSystemNode(Node):
    def __init__(self):
        super().__init__("led_system_node")
        self.subscription = self.create_subscription(
            String, "/car_state", self.car_state_callback, 10
        )
        self.subscription

    def car_state_callback(self, msg):
        state = msg.data
        if state == "GO":
            led_system.GO()
        elif state == "STOP":
            led_system.STOP()
        elif state == "TURN":
            led_system.TURN()
        elif state == "PARK":
            led_system.PARK()


def main(args=None):
    rclpy.init(args=args)
    node = LedSystemNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
