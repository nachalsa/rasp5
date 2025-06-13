# encoding: utf-8
# @date: 2025/06/13
# @author: Gemini
# Mecanum Wheel Auto-Pausing Sequence Node

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import List, Dict, Any

class MecanumControllerNode(Node):
    """
    동작 목록(Action List)을 기반으로 메카넘 휠 로봇을 제어하는 ROS2 노드입니다.
    각 움직임 동작 사이에 자동으로 정지 시간을 추가하여 시퀀스를 실행합니다.
    """
    def __init__(self):
        super().__init__('mecanum_controller_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.get_logger().info('Mecanum Controller Node has been initialized.')
        time.sleep(3)

    def stop_robot(self, duration: float):
        self.get_logger().info(f'Stopping for {duration} seconds...')
        self.cmd_vel_pub.publish(Twist())
        time.sleep(duration)

    def move_straight(self, speed: float, duration: float):
        self.get_logger().info(f'Moving straight for {duration}s at {speed} m/s.')
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

    def move_turn_right(self, linear_speed: float, angular_speed: float, duration: float):
        self.get_logger().info(f'Turning right for {duration}s (linear: {linear_speed} m/s, angular: {angular_speed} rad/s).')
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = -angular_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

    def move_lateral_right(self, speed: float, duration: float):
        self.get_logger().info(f'Moving right for {duration}s at {speed} m/s.')
        twist = Twist()
        twist.linear.y = -speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

    def move_lateral_left(self, speed: float, duration: float):
        self.get_logger().info(f'Moving left for {duration}s at {speed} m/s.')
        twist = Twist()
        twist.linear.y = speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

    def move_backward(self, speed: float, duration: float):
        self.get_logger().info(f'Moving backward for {duration}s at {speed} m/s.')
        twist = Twist()
        twist.linear.x = -speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

    def run_sequence(self, actions: List[Dict[str, Any]], stop_duration: float):
        """
        주어진 동작 목록을 실행하며, 각 움직임 동작 사이에 자동으로 정지 시간을 추가합니다.
        """
        self.get_logger().info(f'Starting action sequence with auto-stopping ({stop_duration}s)...')
        num_actions = len(actions)
        try:
            for i, task in enumerate(actions):
                action_type = task.get('action')
                params = task.get('params', {})
                self.get_logger().info(f"Executing action #{i+1}/{num_actions}: {action_type} with params {params}")

                # 해당하는 동작 실행
                if action_type == 'straight':
                    self.move_straight(**params)
                elif action_type == 'stop':
                    self.stop_robot(**params)
                elif action_type == 'lateral_right':
                    self.move_lateral_right(**params)
                elif action_type == 'lateral_left':
                    self.move_lateral_left(**params)
                elif action_type == 'backward':
                    self.move_backward(**params)
                elif action_type == 'turn_right':
                    self.move_turn_right(**params)
                else:
                    self.get_logger().warn(f"Unknown or non-movement action type: {action_type}")

                # 마지막 동작이 아니면, 설정된 시간만큼 자동으로 정지
                if i < num_actions - 1:
                    self.stop_robot(duration=stop_duration)
            
            self.get_logger().info('Action sequence finished.')

        except TypeError as e:
            self.get_logger().error(f"Parameter mismatch for an action: {e}")
        except Exception as e:
            self.get_logger().error(f'An error occurred during the sequence: {e}')
        finally:
            self.get_logger().info('Ensuring robot is finally stopped.')
            self.stop_robot(duration=1.0)


def main(args=None):
    rclpy.init(args=args)
    controller_node = MecanumControllerNode()

    # =======================================================================
    # ============ 여기서 실행하고 싶은 동작 목록을 자유롭게 구성합니다. ============
    # =======================================================================
    # 이제 'stop' 액션을 수동으로 넣을 필요가 없습니다.
    # 움직임 목록만 순서대로 작성하세요.
    
    action_sequence = [
        {'action': 'straight', 'params': {'speed': 0.3, 'duration': 2.5}},
        {'action': 'stop', 'params': {'duration': 0.2}},
        {'action': 'straight', 'params': {'speed': 0.3, 'duration': 5.0}},
        {'action': 'lateral_right', 'params': {'speed': 0.3, 'duration': 2.0}},
        {'action': 'turn_right', 'params': {'linear_speed': 0.0, 'angular_speed': 0.2, 'duration': 0.1}},
        {'action': 'stop', 'params': {'duration': 0.2}},
        {'action': 'lateral_right', 'params': {'speed': 0.3, 'duration': 6.2}},
        {'action': 'backward', 'params': {'speed': 0.3, 'duration': 4.0}},
        {'action': 'turn_right', 'params': {'linear_speed': 0.0, 'angular_speed': -0.3, 'duration': 0.4}},
        {'action': 'stop', 'params': {'duration': 0.2}},
        {'action': 'backward', 'params': {'speed': 0.3, 'duration': 5.333333333333333}},
        {'action': 'lateral_left', 'params': {'speed': 0.3, 'duration': 2.8}},
        {'action': 'turn_right', 'params': {'linear_speed': 0.0, 'angular_speed': 0.3, 'duration': 0.4}},
        {'action': 'stop', 'params': {'duration': 0.2}},
        {'action': 'lateral_left', 'params': {'speed': 0.3, 'duration': 1.5}},
        {'action': 'straight', 'params': {'speed': 0.3, 'duration': 6.2}},
        {'action': 'lateral_right', 'params': {'speed': 0.3, 'duration': 1.4}},
        {'action': 'stop', 'params': {'duration': 1.0}},
        {'action': 'lateral_left', 'params': {'speed': 0.3, 'duration': 5}},
    ]

    # 모든 움직임 동작 사이에 적용될 정지 시간
    pause_duration = 0.3 

    # 정의한 동작 목록과 정지 시간을 인자로 전달하여 시퀀스 실행
    controller_node.run_sequence(actions=action_sequence, stop_duration=pause_duration)

    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
