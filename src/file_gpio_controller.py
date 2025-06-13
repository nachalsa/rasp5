#!/usr/bin/env python3
# encoding: utf-8
# File-based GPIO Controller - Docker 컨테이너용

import rclpy
import time
import threading
import json
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

# ButtonState 메시지
try:
    from ros_robot_controller_msgs.msg import ButtonState
    BUTTON_STATE_AVAILABLE = True
    print("✅ ButtonState 메시지 로드 성공")
except ImportError:
    BUTTON_STATE_AVAILABLE = False
    print("❌ ButtonState 메시지 없음")

class FileGPIOController(Node):
    """
    파일 기반 GPIO 제어 컨트롤러 (Docker 컨테이너용)
    """
    def __init__(self):
        super().__init__('file_gpio_controller')
        
        # 공유 파일 경로
        self.shared_dir = "/home/ubuntu/shared"
        self.command_file = os.path.join(self.shared_dir, "gpio_commands.json")
        self.status_file = os.path.join(self.shared_dir, "gpio_status.json")
        
        # 디렉토리 확인
        if not os.path.exists(self.shared_dir):
            self.get_logger().error(f"❌ 공유 디렉토리 없음: {self.shared_dir}")
            self.get_logger().error("Docker 볼륨 마운트를 확인하세요")
        else:
            self.get_logger().info(f"✅ 공유 디렉토리 확인: {self.shared_dir}")
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        
        # Subscriber
        if BUTTON_STATE_AVAILABLE:
            self.button_sub = self.create_subscription(
                ButtonState, '/ros_robot_controller/button', 
                self.button_callback, 10)
        
        # 상태 관리
        self.lock = threading.Lock()
        self.is_running = False
        self.should_stop = False
        self.sequence_thread = None
        
        # 액션 시퀀스
        self.actions = [
            {'name': 'Straight 1', 'cmd': [0.3, 0.0, 0.0], 'duration': 2.5},
            {'name': 'Crosswalk Stop', 'cmd': [0.0, 0.0, 0.0], 'duration': 2.0},
            {'name': 'Straight 2', 'cmd': [0.3, 0.0, 0.0], 'duration': 5.0},
            {'name': 'Turn Right', 'cmd': [0.0, 0.0, -0.3], 'duration': 0.3, 'yellow': True},
            {'name': 'Move Right 1', 'cmd': [0.0, -0.3, 0.0], 'duration': 1.8, 'yellow': True},
            {'name': 'Move Right 2', 'cmd': [0.0, -0.3, 0.0], 'duration': 6.0, 'yellow': True},
            {'name': 'Backward 1', 'cmd': [-0.3, 0.0, 0.0], 'duration': 3.6},
            {'name': 'Backward 2', 'cmd': [-0.3, 0.0, 0.0], 'duration': 5.0},
            {'name': 'Move Left 1', 'cmd': [0.0, 0.3, 0.0], 'duration': 2.8, 'yellow': True},
            {'name': 'Move Left 2', 'cmd': [0.0, 0.3, 0.0], 'duration': 1.5, 'yellow': True},
            {'name': 'Straight 3', 'cmd': [0.3, 0.0, 0.0], 'duration': 6.2},
            {'name': 'Move Right 3', 'cmd': [0.0, -0.3, 0.0], 'duration': 1.4, 'yellow': True},
            {'name': 'Traffic Light Stop', 'cmd': [0.0, 0.0, 0.0], 'duration': 3.0},
            {'name': 'Move Left 3', 'cmd': [0.0, 0.3, 0.0], 'duration': 5.0, 'yellow': True},
            {'name': 'Parking', 'cmd': [0.0, 0.0, 0.0], 'duration': 5.0, 'parking': True},
        ]
        
        self.get_logger().info('=== 📄 File GPIO Controller Ready ===')
        self.get_logger().info('🔘 Button 1: Start/Restart | Button 2: Reset')
        
        # 초기 LED 상태 설정
        self.send_gpio_command("set_leds", {"green": False, "red": True})
        
        # LED 테스트 실행
        self.test_leds()

    def send_gpio_command(self, command: str, params: dict = None):
        """GPIO 명령을 파일로 전송"""
        try:
            gpio_command = {
                "timestamp": time.time(),
                "command": command,
                "params": params or {}
            }
            
            with open(self.command_file, 'w') as f:
                json.dump(gpio_command, f, indent=2)
            
            self.get_logger().info(f"📤 GPIO 명령 전송: {command}")
            if params:
                self.get_logger().info(f"   매개변수: {params}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ GPIO 명령 전송 실패: {e}")
            return False

    def get_gpio_status(self):
        """GPIO 상태 확인"""
        try:
            if os.path.exists(self.status_file):
                with open(self.status_file, 'r') as f:
                    status = json.load(f)
                return status
        except Exception as e:
            self.get_logger().error(f"상태 파일 읽기 오류: {e}")
        return None

    def test_leds(self):
        """LED 테스트"""
        def test_worker():
            time.sleep(3)  # 초기화 후 대기
            self.get_logger().info("🧪 LED 테스트 시작")
            self.send_gpio_command("test_leds")
        
        threading.Thread(target=test_worker, daemon=True).start()

    def set_leds(self, moving: bool):
        """LED 상태 설정"""
        params = {
            "green": moving,
            "red": not moving
        }
        
        if self.send_gpio_command("set_leds", params):
            status = "MOVING 🟢" if moving else "STOPPED 🔴"
            self.get_logger().info(f"💡 {status}")

    def start_yellow_blink(self):
        """노란색 LED 점멸 시작"""
        self.send_gpio_command("blink_yellow_start")
        self.get_logger().info("💛 방향지시등 점멸 시작")

    def stop_yellow_blink(self):
        """노란색 LED 점멸 중지"""
        self.send_gpio_command("blink_yellow_stop")
        self.get_logger().info("💛 방향지시등 점멸 중지")

    def start_all_blink(self, duration: float = 5.0):
        """모든 LED 점멸 (주차 완료)"""
        params = {"duration": duration}
        self.send_gpio_command("blink_all_start", params)
        self.get_logger().info(f"✨ 주차 완료 - 모든 LED 점멸 ({duration}초)")

    def button_callback(self, msg):
        """버튼 콜백"""
        button_id = msg.id
        self.get_logger().info(f'🔘 Button {button_id} pressed')
        
        with self.lock:
            running = self.is_running
        
        if button_id == 1:
            if running:
                self.get_logger().info('🔄 RESTART requested')
                self.restart()
            else:
                self.get_logger().info('🚀 START requested')
                self.start()
        elif button_id == 2:
            self.get_logger().info('🏠 RESET requested')
            self.reset()

    def start(self):
        """시퀀스 시작"""
        with self.lock:
            if self.is_running:
                return
            self.is_running = True
            self.should_stop = False
        
        self.sequence_thread = threading.Thread(target=self.run_sequence, daemon=True)
        self.sequence_thread.start()

    def restart(self):
        """재시작"""
        self.stop_sequence()
        time.sleep(0.5)
        self.start()

    def reset(self):
        """리셋"""
        self.stop_sequence()
        self.emergency_stop()

    def stop_sequence(self):
        """시퀀스 중지"""
        with self.lock:
            self.should_stop = True
            self.is_running = False
        
        if self.sequence_thread and self.sequence_thread.is_alive():
            self.sequence_thread.join(timeout=2.0)

    def emergency_stop(self):
        """즉시 정지"""
        self.publish_cmd(0.0, 0.0, 0.0)
        self.set_leds(moving=False)
        self.stop_yellow_blink()

    def run_sequence(self):
        """시퀀스 실행"""
        self.get_logger().info('=== 🎬 SEQUENCE STARTED ===')
        
        try:
            for i, action in enumerate(self.actions):
                with self.lock:
                    if self.should_stop:
                        break
                
                self.get_logger().info(f'[{i+1}/{len(self.actions)}] {action["name"]}')
                
                # 방향지시등
                if action.get('yellow', False):
                    self.start_yellow_blink()
                
                # LED 상태
                moving = any(abs(x) > 0.01 for x in action['cmd'])
                self.set_leds(moving)
                
                # 명령 실행
                self.publish_cmd(*action['cmd'])
                
                # 주차 완료 처리
                if action.get('parking', False):
                    self.start_all_blink(action['duration'])
                    if not self.sleep_check(action['duration']):
                        break
                else:
                    if not self.sleep_check(action['duration']):
                        break
                
                # 방향지시등 끄기
                if action.get('yellow', False):
                    self.stop_yellow_blink()
                
                # 액션 간 대기
                if i < len(self.actions) - 1:
                    if not self.sleep_check(0.2):
                        break
            
            with self.lock:
                stopped = self.should_stop
            
            if not stopped:
                self.get_logger().info('=== ✅ SEQUENCE COMPLETED ===')
            else:
                self.get_logger().info('=== ⏹️ SEQUENCE STOPPED ===')
                
        except Exception as e:
            self.get_logger().error(f'Sequence error: {e}')
        finally:
            with self.lock:
                self.is_running = False
            self.emergency_stop()

    def sleep_check(self, duration: float) -> bool:
        """중단 가능한 대기"""
        start = time.time()
        while time.time() - start < duration:
            with self.lock:
                if self.should_stop:
                    return False
            time.sleep(0.05)
        return True

    def publish_cmd(self, x: float, y: float, z: float):
        """차량 명령 발행"""
        try:
            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = float(y)
            twist.angular.z = float(z)
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'명령 발행 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = FileGPIOController()
        controller.get_logger().info('📄 File GPIO Controller 실행 중...')
        rclpy.spin(controller)
    except Exception as e:
        print(f"오류: {e}")
    except KeyboardInterrupt:
        print('🛑 사용자 중단')
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()

