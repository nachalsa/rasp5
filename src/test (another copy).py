import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import sys

# 이 테스트의 목적: 
# 1. 'control_node'가 'status' 토픽을 발행하는지
# 2. 'sensor_node'가 'status' 토픽을 구독해서 'ack' 토픽을 발행하는지
# 3. 5초 안에 'ack' 토픽을 받으면 성공, 아니면 실패

class IntegrationTesterNode(Node):
    def __init__(self):
        super().__init__('integration_tester')
        self.ack_received = False
        
        self.publisher_ = self.create_publisher(String, 'status', 10)
        self.subscription = self.create_subscription(
            Bool,
            'ack',
            self.ack_callback,
            10)
        
    def ack_callback(self, msg):
        if msg.data:
            self.get_logger().info("SUCCESS: Acknowledgment received!")
            self.ack_received = True

    def run_test(self):
        self.get_logger().info("Starting integration test...")
        # 1. 테스트 메시지 발행
        msg = String()
        msg.data = "PING"
        self.publisher_.publish(msg)
        
        # 2. 5초 동안 응답 대기
        timeout = 5.0 # 초
        start_time = time.time()
        while not self.ack_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 3. 결과 판정
        if self.ack_received:
            self.get_logger().info("Test PASSED.")
            return 0 # 성공 시 종료 코드 0
        else:
            self.get_logger().error("Test FAILED: No acknowledgment received within timeout.")
            return 1 # 실패 시 종료 코드 1

def main(args=None):
    rclpy.init(args=args)
    tester_node = IntegrationTesterNode()
    result = tester_node.run_test()
    
    tester_node.destroy_node()
    rclpy.shutdown()
    
    # 스크립트의 종료 코드를 테스트 결과로 설정
    sys.exit(result)

if __name__ == '__main__':
    main()
