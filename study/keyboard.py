#!/usr/bin/env python3
# encoding: utf-8
# keyboard driving with depth camera display and capture
import os
import cv2
import time
import queue
import rclpy
import threading
import numpy as np
from datetime import datetime
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class KeyboardDriveNode(Node):
    def __init__(self, name):
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.name = name
        self.is_running = True
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        
        # 이미지 큐 설정
        self.rgb_image_queue = queue.Queue(maxsize=2)
        self.depth_image_queue = queue.Queue(maxsize=2)
        
        # 현재 속도 설정
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.current_twist = Twist()
        
        # 키보드 입력 상태
        self.key_pressed = {
            'w': False, 's': False, 'a': False, 'd': False,
            'q': False, 'e': False, 'z': False, 'x': False
        }
        
        # 저장할 이미지 디렉토리 생성
        self.save_dir = "captured_images"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.param_init()
        self.setup_publishers_subscribers()
        
        # 키보드 제어 스레드 시작
        print("키보드 제어 스레드 시작...")
        self.keyboard_thread = threading.Thread(target=self.keyboard_control_loop, daemon=True)
        self.keyboard_thread.start()
        
        # 이미지 디스플레이 스레드 시작  
        print("디스플레이 스레드 시작...")
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        print("모든 스레드 시작 완료")
        
        self.get_logger().info("\033[1;32m키보드 제어 시작!\033[0m")
        self.get_logger().info("조작법:")
        self.get_logger().info("W/S: 전진/후진, A/D: 좌회전/우회전")
        self.get_logger().info("Q/E: 좌측이동/우측이동 (메카넘), Z/X: 좌회전/우회전 제자리")
        self.get_logger().info("C: RGB 이미지 캡처, V: Depth 이미지 캡처")
        self.get_logger().info("SPACE: 즉시 정지, ESC: 종료")
        
        # 테스트 메시지 발행
        self.get_logger().info("테스트 메시지 발행 중...")
        test_twist = Twist()
        self.mecanum_pub.publish(test_twist)
        self.get_logger().info("테스트 메시지 발행 완료")

    def param_init(self):
        self.start = True
        self.enter = True
        self.display = True
        
    def setup_publishers_subscribers(self):
        # Publisher 설정
        self.mecanum_pub = self.create_publisher(Twist, "/controller/cmd_vel", 1)
        
        # Service 설정
        self.create_service(Trigger, "~/enter", self.enter_srv_callback)
        self.create_service(Trigger, "~/exit", self.exit_srv_callback)
        self.create_service(SetBool, "~/set_running", self.set_running_srv_callback)
        
        # Subscriber 설정 - RGB 카메라
        self.rgb_sub = self.create_subscription(
            Image, "/ascamera/camera_publisher/rgb0/image", self.rgb_image_callback, 1
        )
        
        # Subscriber 설정 - Depth 카메라
        self.depth_sub = self.create_subscription(
            Image, "/ascamera/camera_publisher/depth0/image", self.depth_image_callback, 1
        )

    def rgb_image_callback(self, ros_image):
        """RGB 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            if self.rgb_image_queue.full():
                self.rgb_image_queue.get()
            self.rgb_image_queue.put(rgb_image)
        except Exception as e:
            self.get_logger().error(f"RGB 이미지 처리 오류: {e}")

    def depth_image_callback(self, ros_image):
        """Depth 이미지 콜백"""
        try:
            # Depth 이미지는 보통 16bit 또는 32bit float
            if ros_image.encoding == "16UC1":
                cv_image = self.bridge.imgmsg_to_cv2(ros_image, "16UC1")
                # 시각화를 위해 8bit로 변환
                depth_image = cv2.convertScaleAbs(cv_image, alpha=0.03)
            elif ros_image.encoding == "32FC1":
                cv_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
                # NaN 값 처리
                cv_image = np.nan_to_num(cv_image)
                # 시각화를 위해 정규화 및 8bit 변환
                if cv_image.max() > 0:
                    depth_image = cv2.convertScaleAbs(cv_image, alpha=255.0/cv_image.max())
                else:
                    depth_image = np.zeros_like(cv_image, dtype=np.uint8)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(ros_image, ros_image.encoding)
                depth_image = cv2.convertScaleAbs(cv_image, alpha=0.03)
                
            if self.depth_image_queue.full():
                self.depth_image_queue.get()
            self.depth_image_queue.put(depth_image)
        except Exception as e:
            self.get_logger().error(f"Depth 이미지 처리 오류: {e}")

    def keyboard_control_loop(self):
        """키보드 입력을 처리하는 루프"""
        print("키보드 제어 루프 시작")
        self.get_logger().info("키보드 제어 루프 활성화")
        
        while self.is_running:
            try:
                # 현재 입력된 키들을 확인하고 Twist 메시지 생성
                twist = Twist()
                movement_detected = False
                
                # 선형 운동 (전진/후진)
                if self.key_pressed['w']:
                    twist.linear.x = self.linear_speed
                    movement_detected = True
                elif self.key_pressed['s']:
                    twist.linear.x = -self.linear_speed
                    movement_detected = True
                
                # 측면 운동 (메카넘 휠용)
                if self.key_pressed['q']:
                    twist.linear.y = self.linear_speed
                    movement_detected = True
                elif self.key_pressed['e']:
                    twist.linear.y = -self.linear_speed
                    movement_detected = True
                
                # 회전 운동
                if self.key_pressed['a']:
                    twist.angular.z = self.angular_speed
                    movement_detected = True
                elif self.key_pressed['d']:
                    twist.angular.z = -self.angular_speed
                    movement_detected = True
                elif self.key_pressed['z']:
                    twist.angular.z = self.angular_speed
                    movement_detected = True
                elif self.key_pressed['x']:
                    twist.angular.z = -self.angular_speed
                    movement_detected = True
                
                # Twist 메시지 발행
                self.current_twist = twist
                if self.start:
                    self.mecanum_pub.publish(twist)
                    if movement_detected:
                        print(f"움직임: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, z={twist.angular.z:.2f}")
                    
                time.sleep(0.05)  # 20Hz
                
            except Exception as e:
                self.get_logger().error(f"키보드 제어 오류: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)

    def display_loop(self):
        """이미지 디스플레이 루프"""
        print("디스플레이 루프 시작")
        self.get_logger().info("OpenCV 윈도우 초기화 중...")
        
        # 초기 윈도우 생성
        cv2.namedWindow("Control Window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Control Window", 400, 300)
        
        # 초기 화면 생성
        initial_img = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(initial_img, "Waiting for camera...", (50, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Control Window", initial_img)
        
        while self.is_running:
            try:
                rgb_image = None
                depth_image = None
                
                # RGB 이미지 가져오기
                try:
                    rgb_image = self.rgb_image_queue.get(block=False)
                    print("RGB 이미지 수신됨")
                except queue.Empty:
                    pass
                
                # Depth 이미지 가져오기
                try:
                    depth_image = self.depth_image_queue.get(block=False)
                    print("Depth 이미지 수신됨")
                except queue.Empty:
                    pass
                
                # 이미지 디스플레이
                display_image = None
                if rgb_image is not None:
                    # RGB를 BGR로 변환 (OpenCV 디스플레이용)
                    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                    # 현재 속도 정보 오버레이
                    self.add_speed_overlay(bgr_image)
                    cv2.imshow("RGB Camera", bgr_image)
                    display_image = bgr_image
                
                if depth_image is not None:
                    # Depth 이미지에 컬러맵 적용
                    depth_colored = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
                    cv2.imshow("Depth Camera", depth_colored)
                    if display_image is None:
                        display_image = depth_colored
                
                # 메인 컨트롤 윈도우 업데이트
                if display_image is None:
                    # 카메라 없을 때 기본 화면
                    control_img = np.zeros((400, 500, 3), dtype=np.uint8)
                    self.add_control_info(control_img)
                    cv2.imshow("Control Window", control_img)
                
                # 키 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if key != 255:  # 키가 눌렸을 때
                    print(f"키 입력: {chr(key) if 32 <= key <= 126 else key}")
                    self.handle_key_input(key, rgb_image, depth_image)
                
                if key == 27:  # ESC 키
                    self.get_logger().info("ESC 키 눌림 - 종료")
                    self.shutdown()
                    break
                    
            except Exception as e:
                self.get_logger().error(f"디스플레이 루프 오류: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        print("디스플레이 루프 종료")

    def add_control_info(self, image):
        """컨트롤 정보를 이미지에 추가"""
        h, w = image.shape[:2]
        
        # 현재 상태 정보
        status_text = [
            f"=== KEYBOARD CONTROL ===",
            f"Running: {'ON' if self.is_running else 'OFF'}",
            f"Control: {'ON' if self.start else 'OFF'}",
            f"",
            f"Current Speed:",
            f"  Linear X: {self.current_twist.linear.x:.2f}",
            f"  Linear Y: {self.current_twist.linear.y:.2f}",
            f"  Angular Z: {self.current_twist.angular.z:.2f}",
            f"",
            f"Active Keys:",
        ]
        
        # 활성화된 키들 표시
        active_keys = [k for k, v in self.key_pressed.items() if v]
        if active_keys:
            status_text.append(f"  {', '.join(active_keys).upper()}")
        else:
            status_text.append("  None")
            
        status_text.extend([
            f"",
            f"Controls:",
            f"  W/S: Forward/Backward",
            f"  A/D: Turn Left/Right", 
            f"  Q/E: Strafe Left/Right",
            f"  Z/X: Rotate Left/Right",
            f"  SPACE: Stop All",
            f"  C: Capture RGB",
            f"  V: Capture Depth",
            f"  +/-: Speed Up/Down",
            f"  ESC: Exit"
        ])
        
        # 텍스트 그리기
        for i, text in enumerate(status_text):
            y_pos = 25 + i * 20
            if y_pos > h - 10:
                break
            color = (0, 255, 0) if "ON" in text else (255, 255, 255)
            if "Current Speed:" in text or "Active Keys:" in text:
                color = (0, 255, 255)
            cv2.putText(image, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def add_speed_overlay(self, image):
        """이미지에 현재 속도 정보를 오버레이"""
        h, w = image.shape[:2]
        
        # 현재 속도 정보 텍스트
        speed_text = [
            f"Linear X: {self.current_twist.linear.x:.2f}",
            f"Linear Y: {self.current_twist.linear.y:.2f}",
            f"Angular Z: {self.current_twist.angular.z:.2f}",
            "",
            "Controls:",
            "W/S: Forward/Backward",
            "A/D: Turn Left/Right", 
            "Q/E: Strafe Left/Right",
            "Z/X: Rotate Left/Right",
            "C: Capture RGB",
            "V: Capture Depth",
            "ESC: Exit"
        ]
        
        # 텍스트 배경 그리기
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (300, 320), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # 텍스트 그리기
        for i, text in enumerate(speed_text):
            y_pos = 35 + i * 25
            color = (0, 255, 0) if i < 3 else (255, 255, 255)
            cv2.putText(image, text, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def handle_key_input(self, key, rgb_image, depth_image):
        """키 입력 처리"""
        if key == 255:  # 키가 눌리지 않음
            return
            
        key_char = chr(key) if 32 <= key <= 126 else f"Key({key})"
        print(f"키 처리: {key_char}")
        
        # 이동 키 처리 (토글 방식)
        if key == ord('w'):
            self.key_pressed['w'] = not self.key_pressed['w']
            self.get_logger().info(f"전진: {'ON' if self.key_pressed['w'] else 'OFF'}")
        elif key == ord('s'):
            self.key_pressed['s'] = not self.key_pressed['s']
            self.get_logger().info(f"후진: {'ON' if self.key_pressed['s'] else 'OFF'}")
        elif key == ord('a'):
            self.key_pressed['a'] = not self.key_pressed['a']
            self.get_logger().info(f"좌회전: {'ON' if self.key_pressed['a'] else 'OFF'}")
        elif key == ord('d'):
            self.key_pressed['d'] = not self.key_pressed['d']
            self.get_logger().info(f"우회전: {'ON' if self.key_pressed['d'] else 'OFF'}")
        elif key == ord('q'):
            self.key_pressed['q'] = not self.key_pressed['q']
            self.get_logger().info(f"좌측이동: {'ON' if self.key_pressed['q'] else 'OFF'}")
        elif key == ord('e'):
            self.key_pressed['e'] = not self.key_pressed['e']
            self.get_logger().info(f"우측이동: {'ON' if self.key_pressed['e'] else 'OFF'}")
        elif key == ord('z'):
            self.key_pressed['z'] = not self.key_pressed['z']
            self.get_logger().info(f"제자리 좌회전: {'ON' if self.key_pressed['z'] else 'OFF'}")
        elif key == ord('x'):
            self.key_pressed['x'] = not self.key_pressed['x']
            self.get_logger().info(f"제자리 우회전: {'ON' if self.key_pressed['x'] else 'OFF'}")
        
        # 정지 (스페이스바)
        elif key == ord(' '):
            # 모든 키 상태 리셋
            for k in self.key_pressed:
                self.key_pressed[k] = False
            self.mecanum_pub.publish(Twist())
            self.get_logger().info("모든 움직임 정지!")
        
        # 이미지 캡처
        elif key == ord('c'):
            if rgb_image is not None:
                self.capture_image(rgb_image, "rgb")
            else:
                self.get_logger().warn("RGB 이미지가 없습니다")
        elif key == ord('v'):
            if depth_image is not None:
                self.capture_image(depth_image, "depth")
            else:
                self.get_logger().warn("Depth 이미지가 없습니다")
        
        # 속도 조절
        elif key == ord('+') or key == ord('='):
            self.linear_speed = min(1.0, self.linear_speed + 0.1)
            self.angular_speed = min(2.0, self.angular_speed + 0.1)
            self.get_logger().info(f"속도 증가: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}")
        elif key == ord('-'):
            self.linear_speed = max(0.1, self.linear_speed - 0.1)
            self.angular_speed = max(0.1, self.angular_speed - 0.1)
            self.get_logger().info(f"속도 감소: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}")
            
        # 제어 활성화/비활성화
        elif key == ord('t'):
            self.start = not self.start
            self.get_logger().info(f"제어 {'활성화' if self.start else '비활성화'}")
            if not self.start:
                self.mecanum_pub.publish(Twist())  # 정지"속도 감소: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}")

    def capture_image(self, image, image_type):
        """이미지 캡처 및 저장"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.save_dir}/{image_type}_{timestamp}.png"
            
            if image_type == "rgb":
                # RGB 이미지를 BGR로 변환해서 저장
                bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.imwrite(filename, bgr_image)
            else:
                # Depth 이미지 저장
                cv2.imwrite(filename, image)
            
            self.get_logger().info(f"{image_type.upper()} 이미지 저장: {filename}")
            
        except Exception as e:
            self.get_logger().error(f"이미지 저장 오류: {e}")

    def enter_srv_callback(self, request, response):
        self.get_logger().info("키보드 제어 활성화")
        with self.lock:
            self.start = True
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info("키보드 제어 비활성화")
        with self.lock:
            self.start = False
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info(f"실행 상태 변경: {request.data}")
        with self.lock:
            self.start = request.data
            if not self.start:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

    def shutdown(self):
        """종료 처리"""
        self.get_logger().info("시스템 종료 중...")
        self.is_running = False
        with self.lock:
            self.mecanum_pub.publish(Twist())  # 정지
        cv2.destroyAllWindows()


def main():
    # ROS2 초기화
    if not rclpy.ok():
        rclpy.init()
    
    try:
        print("키보드 제어 노드 시작 중...")
        node = KeyboardDriveNode("keyboard_drive")
        print("노드 생성 완료")
        
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        print("Executor 설정 완료")
        
        # 초기 상태 확인
        node.get_logger().info("=== 시스템 상태 ===")
        node.get_logger().info(f"노드 실행 중: {node.is_running}")
        node.get_logger().info(f"제어 활성화: {node.start}")
        node.get_logger().info("==================")
        
        # 메인 루프
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Ctrl+C 감지, 종료 중...")
        except Exception as e:
            node.get_logger().error(f"Executor 오류: {e}")
                
    except Exception as e:
        print(f"노드 실행 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'node' in locals():
                node.shutdown()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"종료 중 오류: {e}")


if __name__ == "__main__":
    main()