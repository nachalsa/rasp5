#!/usr/bin/env python3
"""
GPIO 데몬 - 방향 LED + 기본 LED 통합 제어 (수정된 핀 맵핑)
파일: gpio_daemon.py
위치: /home/car/gpio_daemon.py (호스트에서 실행)
"""

import json
import time
import threading
import RPi.GPIO as GPIO
from datetime import datetime
from pathlib import Path
import sys

class GPIODaemon:
    def __init__(self):
        # 🎯 LED GPIO 핀 맵핑 (정확한 핀 번호)
        self.led_pins = {
            '1': 18,         # 우측 뒤 (GPIO 18)
            '2': 23,         # 우측 앞 (GPIO 23)
            '3': 24,         # 좌측 앞 (GPIO 24)
            '4': 25,         # 좌측 뒤 (GPIO 25)
            'green': 20,     # 녹색 LED (GPIO 20) - 차량 움직임
            'red': 21,       # 빨간색 LED (GPIO 21) - 차량 정지
        }
        
        # 파일 경로 설정
        self.command_file = Path("/home/car/docker/tmp/gpio_commands.json")
        self.status_file = Path("/home/car/docker/tmp/gpio_status.json")
        
        # LED 상태 관리 (GPIO 핀별)
        self.led_states = {pin: False for pin in self.led_pins.values()}
        self.blink_states = {pin: False for pin in self.led_pins.values()}
        self.blink_threads = {}
        
        # 초기화
        self.setup_gpio()
        self.update_status_file()
        
    def setup_gpio(self):
        """GPIO 핀 초기 설정"""
        print("🔧 GPIO 초기화 중...")
        print("📍 차량 방향 LED 레이아웃:")
        print("     ## 3  2  (앞쪽)")
        print("     ## 4  1  (뒤쪽)")
        print("🚦 기본 LED: 녹색(움직임), 빨간색(정지)")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 모든 LED 핀을 출력으로 설정
        for led_name, pin_num in self.led_pins.items():
            GPIO.setup(pin_num, GPIO.OUT)
            GPIO.output(pin_num, GPIO.LOW)
            if led_name in ['1', '2', '3', '4']:
                print(f"   방향 LED {led_name}: GPIO {pin_num} 설정 완료")
            else:
                print(f"   기본 LED {led_name}: GPIO {pin_num} 설정 완료")
            
        print("✅ GPIO 초기화 완료")
        
    def get_gpio_pin(self, led_name):
        """LED 이름으로 실제 GPIO 핀 번호 가져오기"""
        return self.led_pins.get(led_name)
        
    def led_on(self, led_name):
        """LED 켜기"""
        gpio_pin = self.get_gpio_pin(led_name)
        if gpio_pin is not None:
            # 해당 핀의 점멸 중지
            self.stop_blink_by_pin(gpio_pin)
            # LED 켜기
            GPIO.output(gpio_pin, GPIO.HIGH)
            self.led_states[gpio_pin] = True
            print(f"💡 {led_name} LED ON (GPIO {gpio_pin})")
            return True
        print(f"❌ 알 수 없는 LED: {led_name}")
        return False
        
    def led_off(self, led_name):
        """LED 끄기"""
        gpio_pin = self.get_gpio_pin(led_name)
        if gpio_pin is not None:
            # 해당 핀의 점멸 중지
            self.stop_blink_by_pin(gpio_pin)
            # LED 끄기
            GPIO.output(gpio_pin, GPIO.LOW)
            self.led_states[gpio_pin] = False
            print(f"⚫ {led_name} LED OFF (GPIO {gpio_pin})")
            return True
        print(f"❌ 알 수 없는 LED: {led_name}")
        return False
        
    def start_blink(self, led_name, interval=0.5):
        """LED 점멸 시작"""
        gpio_pin = self.get_gpio_pin(led_name)
        if gpio_pin is None:
            print(f"❌ 알 수 없는 LED: {led_name}")
            return False
            
        # 기존 점멸 중지
        self.stop_blink_by_pin(gpio_pin)
        
        # 점멸 상태 설정
        self.blink_states[gpio_pin] = True
        
        # 점멸 스레드 시작
        def blink_worker():
            print(f"💫 {led_name} LED 점멸 시작 (GPIO {gpio_pin}, 간격: {interval}s)")
            
            while self.blink_states[gpio_pin]:
                GPIO.output(gpio_pin, GPIO.HIGH)
                self.led_states[gpio_pin] = True
                time.sleep(interval)
                
                if self.blink_states[gpio_pin]:  # 중간에 중지되었는지 확인
                    GPIO.output(gpio_pin, GPIO.LOW)
                    self.led_states[gpio_pin] = False
                    time.sleep(interval)
                    
        self.blink_threads[gpio_pin] = threading.Thread(target=blink_worker, daemon=True)
        self.blink_threads[gpio_pin].start()
        return True
        
    def stop_blink_by_pin(self, gpio_pin):
        """GPIO 핀별 점멸 중지"""
        if gpio_pin in self.blink_states:
            self.blink_states[gpio_pin] = False
            if gpio_pin in self.blink_threads:
                self.blink_threads[gpio_pin].join(timeout=1.0)
                del self.blink_threads[gpio_pin]
                
    def stop_blink(self, led_name):
        """LED 점멸 중지"""
        gpio_pin = self.get_gpio_pin(led_name)
        if gpio_pin is not None:
            self.stop_blink_by_pin(gpio_pin)
            
    def all_leds_off(self):
        """모든 LED 끄기"""
        print("🔴 모든 LED 끄기")
        for gpio_pin in self.led_pins.values():
            self.stop_blink_by_pin(gpio_pin)
            GPIO.output(gpio_pin, GPIO.LOW)
            self.led_states[gpio_pin] = False
            
    def all_leds_blink(self, interval=0.3):
        """모든 LED 점멸"""
        print("✨ 모든 LED 점멸 시작")
        for gpio_pin in self.led_pins.values():
            self.blink_states[gpio_pin] = True
            
            def blink_worker(pin):
                while self.blink_states[pin]:
                    GPIO.output(pin, GPIO.HIGH)
                    self.led_states[pin] = True
                    time.sleep(interval)
                    if self.blink_states[pin]:
                        GPIO.output(pin, GPIO.LOW)
                        self.led_states[pin] = False
                        time.sleep(interval)
                        
            self.blink_threads[gpio_pin] = threading.Thread(
                target=blink_worker, args=(gpio_pin,), daemon=True)
            self.blink_threads[gpio_pin].start()
            
    def set_vehicle_state(self, moving=True):
        """차량 상태 설정 (움직임/정지)"""
        if moving:
            print("🟢 차량 움직임 - 녹색 LED ON")
            self.led_off('red')    # 빨간색 끄기
            self.led_on('green')   # 녹색 켜기
        else:
            print("🔴 차량 정지 - 빨간색 LED ON")
            self.led_off('green')  # 녹색 끄기
            self.led_on('red')     # 빨간색 켜기
            
    def test_leds(self):
        """LED 테스트 시퀀스"""
        print("🧪 LED 테스트 시작...")
        
        # 1. 모든 LED 끄기
        self.all_leds_off()
        time.sleep(1)
        
        # 2. 기본 LED 테스트
        print("🚦 기본 LED 테스트")
        print("  - 차량 정지 상태")
        self.set_vehicle_state(moving=False)
        time.sleep(2)
        
        print("  - 차량 움직임 상태")
        self.set_vehicle_state(moving=True)
        time.sleep(2)
        
        # 3. 방향 LED 테스트
        print("🔄 방향 LED 테스트")
        self.set_vehicle_state(moving=False)  # 정지 상태에서 테스트
        
        direction_names = {
            '1': '우측 뒤',
            '2': '우측 앞', 
            '3': '좌측 앞',
            '4': '좌측 뒤'
        }
        
        for led_id, direction in direction_names.items():
            print(f"  - {direction} (LED {led_id})")
            self.led_on(led_id)
            time.sleep(1)
            self.led_off(led_id)
            time.sleep(0.3)
            
        # 4. 그룹별 테스트
        print("🔄 방향 그룹 테스트")
        print("  - 앞쪽 LED (2, 3)")
        self.led_on('2')
        self.led_on('3')
        time.sleep(2)
        self.led_off('2')
        self.led_off('3')
        time.sleep(0.5)
        
        print("  - 뒤쪽 LED (1, 4)")
        self.led_on('1')
        self.led_on('4')
        time.sleep(2)
        self.led_off('1')
        self.led_off('4')
        time.sleep(0.5)
        
        # 5. 점멸 테스트
        print("💫 점멸 테스트")
        self.start_blink('2', 0.3)
        self.start_blink('3', 0.3)
        time.sleep(3)
        self.all_leds_off()
        
        # 6. 최종 상태 - 정지
        self.set_vehicle_state(moving=False)
        
        print("✅ LED 테스트 완료")
        
    def process_command(self, command_data):
        """JSON 명령 처리"""
        try:
            cmd = command_data.get('command')
            params = command_data.get('params', {})
            
            print(f"📨 명령 수신: {cmd}")
            
            if cmd == 'led_on':
                led_name = params.get('led')
                return self.led_on(led_name)
                
            elif cmd == 'led_off':
                led_name = params.get('led')
                return self.led_off(led_name)
                
            elif cmd == 'start_blink':
                led_name = params.get('led')
                interval = params.get('interval', 0.5)
                return self.start_blink(led_name, interval)
                
            elif cmd == 'stop_blink':
                led_name = params.get('led')
                self.stop_blink(led_name)
                return True
                
            elif cmd == 'all_leds_off':
                self.all_leds_off()
                return True
                
            elif cmd == 'all_leds_blink':
                interval = params.get('interval', 0.3)
                self.all_leds_blink(interval)
                return True
                
            elif cmd == 'test_leds':
                self.test_leds()
                return True
                
            # 차량 상태 제어
            elif cmd == 'set_vehicle_moving':
                self.set_vehicle_state(moving=True)
                return True
                
            elif cmd == 'set_vehicle_stopped':
                self.set_vehicle_state(moving=False)
                return True
                
            # 방향별 그룹 제어
            elif cmd == 'front_leds_blink':
                interval = params.get('interval', 0.3)
                self.start_blink('2', interval)  # 우측 앞
                self.start_blink('3', interval)  # 좌측 앞
                return True
                
            elif cmd == 'rear_leds_blink':
                interval = params.get('interval', 0.3)
                self.start_blink('1', interval)  # 우측 뒤
                self.start_blink('4', interval)  # 좌측 뒤
                return True
                
            elif cmd == 'left_leds_blink':
                interval = params.get('interval', 0.3)
                self.start_blink('3', interval)  # 좌측 앞
                self.start_blink('4', interval)  # 좌측 뒤
                return True
                
            elif cmd == 'right_leds_blink':
                interval = params.get('interval', 0.3)
                self.start_blink('1', interval)  # 우측 뒤
                self.start_blink('2', interval)  # 우측 앞
                return True
                
            elif cmd == 'vehicle_sequence':
                sequence_name = params.get('sequence', 'default')
                return self.run_vehicle_sequence(sequence_name)
                
            else:
                print(f"❌ 알 수 없는 명령: {cmd}")
                return False
                
        except Exception as e:
            print(f"❌ 명령 처리 오류: {e}")
            return False
            
    def run_vehicle_sequence(self, sequence_name):
        """차량 제어 시퀀스 실행"""
        print(f"🚗 차량 시퀀스 실행: {sequence_name}")
        
        if sequence_name == 'parking':
            # 주차 시퀀스 
            sequences = [
                ("직진 1", [("vehicle", "moving"), ("front", "blink")], 2.5),
                ("횡단보도 정지", [("vehicle", "stopped")], 3.0),
                ("직진 2", [("vehicle", "moving"), ("front", "blink")], 5.0),
                ("우측 이동", [("vehicle", "moving"), ("right", "blink")], 8.2),
                ("후진", [("vehicle", "moving"), ("rear", "blink")], 9.333),
                ("좌측 이동", [("vehicle", "moving"), ("left", "blink")], 4.3),
                ("직진 3", [("vehicle", "moving"), ("front", "blink")], 6.3),
                ("신호 대기", [("vehicle", "stopped")], 3.0),
                ("최종 좌측 이동", [("vehicle", "moving"), ("left", "blink")], 5.0),
                ("주차 완료", [("all", "blink")], 5.0)
            ]
            
        else:  # default
            sequences = [
                ("시작", [("vehicle", "moving")], 1),
                ("방향 표시", [("front", "blink")], 2),
                ("정지", [("vehicle", "stopped")], 2),
                ("완료", [("all", "blink")], 2)
            ]
            
        # 시퀀스 실행
        for step_name, actions, duration in sequences:
            print(f"  단계: {step_name}")
            
            for action_type, action in actions:
                if action_type == "vehicle":
                    if action == "moving":
                        self.set_vehicle_state(moving=True)
                    elif action == "stopped":
                        self.set_vehicle_state(moving=False)
                elif action_type == "front":
                    if action == "blink":
                        self.start_blink('2', 0.3)
                        self.start_blink('3', 0.3)
                elif action_type == "rear":
                    if action == "blink":
                        self.start_blink('1', 0.3)
                        self.start_blink('4', 0.3)
                elif action_type == "left":
                    if action == "blink":
                        self.start_blink('3', 0.3)
                        self.start_blink('4', 0.3)
                elif action_type == "right":
                    if action == "blink":
                        self.start_blink('1', 0.3)
                        self.start_blink('2', 0.3)
                elif action_type == "all":
                    if action == "blink":
                        self.all_leds_blink(0.2)
                        
            time.sleep(duration)
            
            # 방향 LED만 끄기 (기본 LED는 차량 상태 유지)
            for led_id in ['1', '2', '3', '4']:
                self.stop_blink(led_id)
                self.led_off(led_id)
            
        # 시퀀스 완료 후 정지 상태
        self.set_vehicle_state(moving=False)
        print("✅ 시퀀스 완료")
        return True
        
    def update_status_file(self):
        """상태 파일 업데이트"""
        # LED 이름별 상태로 변환
        led_status = {}
        for led_name, gpio_pin in self.led_pins.items():
            led_status[led_name] = self.led_states.get(gpio_pin, False)
            
        status = {
            'timestamp': int(time.time()),
            'led_states': led_status,
            'gpio_pins': self.led_pins.copy(),
            'direction_layout': {
                'front': ['2', '3'],
                'rear': ['1', '4'], 
                'left': ['3', '4'],
                'right': ['1', '2']
            }
        }
        
        try:
            with open(self.status_file, 'w') as f:
                json.dump(status, f, indent=2)
        except Exception as e:
            print(f"⚠️ 상태 파일 쓰기 오류: {e}")
            
    def monitor_commands(self):
        """명령 파일 모니터링"""
        print("👀 명령 파일 모니터링 시작...")
        print(f"   명령 파일: {self.command_file}")
        print(f"   상태 파일: {self.status_file}")
        print("   (Ctrl+C로 종료)")
        
        last_modified = 0
        
        while True:
            try:
                if self.command_file.exists():
                    current_modified = self.command_file.stat().st_mtime
                    
                    if current_modified > last_modified:
                        # 파일이 수정됨
                        time.sleep(0.1)  # 파일 쓰기 완료 대기
                        
                        try:
                            with open(self.command_file, 'r') as f:
                                command_data = json.load(f)
                                
                            # 명령 처리
                            success = self.process_command(command_data)
                            
                            # 상태 파일 업데이트
                            self.update_status_file()
                            
                            last_modified = current_modified
                            
                        except json.JSONDecodeError as e:
                            print(f"⚠️ JSON 파싱 오류: {e}")
                        except Exception as e:
                            print(f"⚠️ 파일 처리 오류: {e}")
                            
                time.sleep(0.1)  # CPU 사용량 최적화
                
            except KeyboardInterrupt:
                print("\n🛑 종료 신호 수신")
                break
            except Exception as e:
                print(f"⚠️ 모니터링 오류: {e}")
                time.sleep(1)
                
    def cleanup(self):
        """정리 작업"""
        print("🧹 GPIO 정리 중...")
        
        # 모든 점멸 중지
        for gpio_pin in self.led_pins.values():
            self.stop_blink_by_pin(gpio_pin)
            
        # 모든 LED 끄기
        self.all_leds_off()
        
        # GPIO 정리
        GPIO.cleanup()
        print("✅ 정리 완료")

def main():
    daemon = GPIODaemon()
    
    try:
        # 시작 시 LED 테스트
        print("🚀 GPIO 데몬 시작 (방향 LED + 기본 LED)")
        daemon.test_leds()
        
        # 명령 모니터링 시작
        daemon.monitor_commands()
        
    except KeyboardInterrupt:
        print("\n👋 사용자 종료 요청")
    except Exception as e:
        print(f"❌ 예상치 못한 오류: {e}")
    finally:
        daemon.cleanup()
        
if __name__ == "__main__":
    main()

