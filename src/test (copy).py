# test.py
import time
from datetime import datetime

log_file_path = "/tmp/test_log.txt"

print("--- 자율주행 테스트 코드 시작 ---")

with open(log_file_path, "a") as f:
    f.write(f"스크립트 시작: {datetime.now()}\n")

count = 0
while True:
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    message = f"[{count}] 자율주행 중... 현재 시각: {current_time}"
    print(message)
    
    with open(log_file_path, "a") as f:
        f.write(message + "\n")
        
    count += 1
    time.sleep(1)
