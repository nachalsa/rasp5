#!/usr/bin/env python3
# encoding: utf-8
# Combined Mecanum Controller - 버튼 제어 + 완전한 동작 시퀀스

import rclpy
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32MultiArray
from typing import List, Dict, Any

# ButtonState 메시지 타입 시도
try:
    from ros_robot_controller_msgs.msg import ButtonState
    BUTTON_STATE_AVAILABLE = True
    print("✅ ButtonState message loaded")
except ImportError:
    BUTTON_STATE_AVAILABLE = False
    print("❌ ButtonState not found, using fallback")

class CombinedMecanumController(Node):
    """
    완전한 메카넘 휠 컨트롤러 - 버튼 제어 + 확장된 동작 시퀀스
    """
    def __init__(self):
        super().__init__('combined_mecanum_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.led_pub = self.create_publisher(String, '/led_control', 10)
        self.green_led_pub = self.create_publisher(Bool, '/gpio/green_led', 10)
        self.red_led_pub = self.create_publisher(Bool, '/gpio/red_led', 10)
        self.yellow_led_pub = self.create_publisher(Bool, '/gpio/yellow_led', 10)
        
        # Subscribers - 메시지 타입 자동 선택
        try:
            if BUTTON_STATE_AVAILABLE:
                self.button_sub = self.create_subscription(
                    ButtonState, '/ros_robot_controller/button', 
                    self.button_state_callback, 10)
                self.get_logger().info("📡 Subscribed to ButtonState messages")
            else:
                self.button_sub = self.create_subscription(
                    Int32MultiArray, '/ros_robot_controller/button',
                    self.button_array_callback, 10)
                self.get_logger().info("📡 Subscribed to Int32MultiArray messages")
                
        except Exception as e:
            self.get_logger().error(f"Subscription failed: {e}")
            try:
                self.button_sub = self.create_subscription(
                    Int32MultiArray, '/ros_robot_controller/button',
                    self.button_array_callback, 10)
                self.get_logger().info("📡 Fallback: Subscribed to Int32MultiArray messages")
            except Exception as e2:
                self.get_logger().error(f"Both subscription attempts failed: {e2}")
                self.get_logger().warn("⚠️ No button subscription active - use manual commands")
        
        # 상태 관리 (Lock 사용으로 스레드 안전)
        self._state_lock = threading.Lock()
        self._is_running = False
        self._should_stop = False
        self._current_action = 0
        self._sequence_thread = None
        
        # 확장된 액션 시퀀스 (두 번째 코드의 동작들 포함)
        self.actions = [
            {'action': 'straight', 'params': {'speed': 0.3, 'duration': 2.5}},
            {'action': 'stop', 'params': {'duration': 0.2}, 'name': 'Brief Pause'},
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
            {'action': 'lateral_right', 'params': {'speed': 0.3, 'duration': 1.2}},
            {'action': 'stop', 'params': {'duration': 3.0}, 'name': 'Traffic Light'},
            {'action': 'lateral_left', 'params': {'speed': 0.3, 'duration': 5.0}},
            {'action': 'parking', 'params': {'duration': 5.0}, 'name': 'Parking Complete'},
        ]
        
        # 동작 간 자동 정지 시간
        self.pause_duration = 0.3
        
        self.get_logger().info('=== 🎮 Combined Mecanum Controller Ready ===')
        self.get_logger().info(f'📋 Total actions: {len(self.actions)}')
        self.get_logger().info('🔘 Button 1: Start/Restart | Button 2: Reset/Stop')
        
        # 초기 LED 상태
        self.set_basic_leds(moving=False)
        
        # 초기화 대기
        time.sleep(3)

    # === 버튼 콜백 함수들 ===
    def button_state_callback(self, msg):
        """ButtonState 메시지 처리"""
        self.get_logger().info(f'📥 ButtonState received: id={msg.id}, state={msg.state}')
        self.process_button(msg.id, msg.state)

    def button_array_callback(self, msg):
        """Int32MultiArray 메시지 처리 (백업)"""
        if len(msg.data) >= 2:
            button_id = msg.data[0]
            button_state = msg.data[1]
            self.get_logger().info(f'📥 ButtonArray received: id={button_id}, state={button_state}')
            self.process_button(button_id, button_state)

    def process_button(self, button_id: int, button_state: int):
        """통합 버튼 처리 로직"""
        with self._state_lock:
            current_running = self._is_running
        
        self.get_logger().info(f'🎯 Button {button_id} pressed (state={button_state})')
        self.get_logger().info(f'🎯 Current status: {"RUNNING" if current_running else "IDLE"}')
        
        if button_id == 1:
            # Button 1: 시작 또는 재시작
            if current_running:
                self.get_logger().info('🔄 === RESTARTING SEQUENCE ===')
                self.restart_sequence()
            else:
                self.get_logger().info('🚀 === STARTING NEW SEQUENCE ===')
                self.start_sequence()
                
        elif button_id == 2:
            # Button 2: 리셋 (강제 정지)
            self.get_logger().info('🏠 === RESET REQUESTED ===')
            self.reset_sequence()
        else:
            self.get_logger().warn(f'❓ Unknown button ID: {button_id}')

    # === 시퀀스 제어 함수들 ===
    def start_sequence(self):
        """새 시퀀스 시작"""
        with self._state_lock:
            if self._is_running:
                self.get_logger().warn('⚠️ Sequence already running!')
                return
            
            self._is_running = True
            self._should_stop = False
            self._current_action = 0
        
        # 새 스레드에서 시퀀스 실행
        self._sequence_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._sequence_thread.start()
        self.get_logger().info('✅ Sequence thread started')

    def restart_sequence(self):
        """현재 시퀀스 중지 후 재시작"""
        self.get_logger().info('⏹️ Stopping current sequence...')
        
        # 현재 시퀀스 중지
        with self._state_lock:
            self._should_stop = True
        
        # 스레드 종료 대기 (최대 2초)
        if self._sequence_thread and self._sequence_thread.is_alive():
            self._sequence_thread.join(timeout=2.0)
        
        # 즉시 정지
        self.emergency_stop()
        time.sleep(0.5)
        
        # 새 시퀀스 시작
        self.get_logger().info('🔄 Starting fresh sequence...')
        self.start_sequence()

    def reset_sequence(self):
        """완전 리셋 (정지 후 대기 상태)"""
        self.get_logger().info('🛑 Emergency stop requested...')
        
        # 시퀀스 중지
        with self._state_lock:
            self._should_stop = True
            self._is_running = False
        
        # 스레드 종료 대기
        if self._sequence_thread and self._sequence_thread.is_alive():
            self._sequence_thread.join(timeout=2.0)
        
        # 완전 정지
        self.emergency_stop()
        self.get_logger().info('✅ Reset complete - Ready for new sequence')

    def emergency_stop(self):
        """즉시 모든 동작 정지"""
        try:
            # 모든 움직임 정지
            self._publish_twist(0.0, 0.0, 0.0)
            
            # LED를 정지 상태로
            self.set_basic_leds(moving=False)
            self.yellow_off()
            
            self.get_logger().info('🛑 Emergency stop applied')
            
        except Exception as e:
            self.get_logger().error(f'Emergency stop error: {e}')

    # === 시퀀스 실행 (프라이빗 메서드) ===
    def _run_sequence(self):
        """시퀀스 실행 스레드 (내부 메서드)"""
        self.get_logger().info('=== 🎬 SEQUENCE THREAD STARTED ===')
        
        try:
            for i, action in enumerate(self.actions):
                with self._state_lock:
                    if self._should_stop:
                        break
                    self._current_action = i + 1
                
                action_name = action.get('name', action['action'])
                self.get_logger().info(f'[{i+1}/{len(self.actions)}] {action_name}')
                
                # 액션 실행
                if not self._execute_action(action):
                    break
                
                # 중단 체크
                with self._state_lock:
                    if self._should_stop:
                        break
                
                # 액션 간 자동 정지 (마지막 액션이 아니고, 현재 액션이 'stop'이 아닌 경우)
                if i < len(self.actions) - 1 and action['action'] != 'stop':
                    if not self._auto_pause():
                        break
            
            # 완료 처리
            with self._state_lock:
                stopped = self._should_stop
            
            if not stopped:
                self.get_logger().info('=== ✅ SEQUENCE COMPLETED ===')
            else:
                self.get_logger().info('=== ⏹️ SEQUENCE STOPPED ===')
                
        except Exception as e:
            self.get_logger().error(f'Sequence execution error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            # 스레드 종료 시 정리
            with self._state_lock:
                self._is_running = False
                self._should_stop = False
            self.emergency_stop()

    def _execute_action(self, action: Dict) -> bool:
        """개별 액션 실행 (내부 메서드)"""
        with self._state_lock:
            if self._should_stop:
                return False
        
        action_type = action['action']
        params = action.get('params', {})
        
        try:
            if action_type == 'straight':
                return self._move_straight(**params)
            elif action_type == 'backward':
                return self._move_backward(**params)
            elif action_type == 'lateral_left':
                return self._move_lateral_left(**params)
            elif action_type == 'lateral_right':
                return self._move_lateral_right(**params)
            elif action_type == 'turn_right':
                return self._move_turn_right(**params)
            elif action_type == 'stop':
                return self._pause(**params)
            elif action_type == 'parking':
                return self._parking_complete(**params)
            else:
                self.get_logger().warn(f'Unknown action: {action_type}')
                return False
                
        except TypeError as e:
            self.get_logger().error(f"Parameter mismatch for action {action_type}: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f'Action {action_type} failed: {e}')
            return False

    # === 동작 함수들 (프라이빗) ===
    def _move_straight(self, speed: float, duration: float) -> bool:
        self.get_logger().info(f'⬆️ Straight {speed}m/s for {duration}s')
        self.set_basic_leds(moving=True)
        self._publish_twist(speed, 0.0, 0.0)
        return self._sleep_with_check(duration)

    def _move_backward(self, speed: float, duration: float) -> bool:
        self.get_logger().info(f'⬇️ Backward {speed}m/s for {duration}s')
        self.set_basic_leds(moving=True)
        self._publish_twist(-speed, 0.0, 0.0)
        return self._sleep_with_check(duration)

    def _move_lateral_left(self, speed: float, duration: float) -> bool:
        self.get_logger().info(f'⬅️ Left {speed}m/s for {duration}s')
        self.set_basic_leds(moving=True)
        self.yellow_on()
        self._publish_twist(0.0, speed, 0.0)
        result = self._sleep_with_check(duration)
        self.yellow_off()
        return result

    def _move_lateral_right(self, speed: float, duration: float) -> bool:
        self.get_logger().info(f'➡️ Right {speed}m/s for {duration}s')
        self.set_basic_leds(moving=True)
        self.yellow_on()
        self._publish_twist(0.0, -speed, 0.0)
        result = self._sleep_with_check(duration)
        self.yellow_off()
        return result

    def _move_turn_right(self, linear_speed: float, angular_speed: float, duration: float) -> bool:
        self.get_logger().info(f'↗️ Turn right for {duration}s (linear: {linear_speed}, angular: {angular_speed})')
        self.set_basic_leds(moving=True)
        self.yellow_on()
        self._publish_twist(linear_speed, 0.0, -angular_speed)
        result = self._sleep_with_check(duration)
        self.yellow_off()
        return result

    def _pause(self, duration: float) -> bool:
        self.get_logger().info(f'⏸️ Pause for {duration}s')
        self.set_basic_leds(moving=False)
        self._publish_twist(0.0, 0.0, 0.0)
        return self._sleep_with_check(duration)

    def _auto_pause(self) -> bool:
        """자동 정지 (액션 간)"""
        self.get_logger().info(f'⏸️ Auto pause for {self.pause_duration}s')
        self.set_basic_leds(moving=False)
        self._publish_twist(0.0, 0.0, 0.0)
        return self._sleep_with_check(self.pause_duration)

    def _parking_complete(self, duration: float) -> bool:
        self.get_logger().info('🅿️ PARKING COMPLETED!')
        self.set_basic_leds(moving=False)
        self._publish_twist(0.0, 0.0, 0.0)
        
        # 모든 LED 점멸
        blink_count = int(duration * 2)  # 0.5초 간격
        for i in range(blink_count):
            with self._state_lock:
                if self._should_stop:
                    break
            
            self.all_leds_on()
            if not self._sleep_with_check(0.25):
                break
            self.all_leds_off()
            if not self._sleep_with_check(0.25):
                break
        
        self.set_basic_leds(moving=False)
        return True

    # === 유틸리티 함수들 ===
    def _sleep_with_check(self, duration: float) -> bool:
        """중단 가능한 sleep - True: 정상 완료, False: 중단됨"""
        start_time = time.time()
        while time.time() - start_time < duration:
            with self._state_lock:
                if self._should_stop:
                    return False
            time.sleep(0.05)  # 50ms 간격으로 체크
        return True

    def _publish_twist(self, x: float, y: float, z: float):
        """Twist 메시지 발행 (내부 메서드)"""
        try:
            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = float(y)
            twist.angular.z = float(z)
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Twist publish error: {e}')

    # === LED 제어 함수들 ===
    def set_basic_leds(self, moving: bool):
        """기본 LED 상태"""
        try:
            green_msg = Bool()
            green_msg.data = moving
            self.green_led_pub.publish(green_msg)
            
            red_msg = Bool()
            red_msg.data = not moving
            self.red_led_pub.publish(red_msg)
            
        except Exception as e:
            self.get_logger().error(f'LED control error: {e}')

    def yellow_on(self):
        """방향지시등 켜기"""
        try:
            msg = Bool()
            msg.data = True
            self.yellow_led_pub.publish(msg)
        except:
            pass

    def yellow_off(self):
        """방향지시등 끄기"""
        try:
            msg = Bool()
            msg.data = False
            self.yellow_led_pub.publish(msg)
        except:
            pass

    def all_leds_on(self):
        """모든 LED 켜기"""
        try:
            msg = Bool()
            msg.data = True
            self.green_led_pub.publish(msg)
            self.red_led_pub.publish(msg)
            self.yellow_led_pub.publish(msg)
        except:
            pass

    def all_leds_off(self):
        """모든 LED 끄기"""
        try:
            msg = Bool()
            msg.data = False
            self.green_led_pub.publish(msg)
            self.red_led_pub.publish(msg)
            self.yellow_led_pub.publish(msg)
        except:
            pass

    # === 상태 조회 ===
    def get_status(self) -> str:
        """현재 상태 반환"""
        with self._state_lock:
            if self._is_running:
                return f"Running (Action {self._current_action}/{len(self.actions)})"
            else:
                return "Ready"


def main(args=None):
    rclpy.init(args=args)
    controller = CombinedMecanumController()
    
    try:
        controller.get_logger().info('🎮 Combined Controller spinning... Press buttons to control!')
        controller.get_logger().info(f'📋 Loaded {len(controller.actions)} actions in sequence')
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Interrupted by user')
    finally:
        controller.reset_sequence()  # 정리
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

