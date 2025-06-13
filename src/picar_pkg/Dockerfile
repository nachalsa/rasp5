# 라즈베리파이에서 미리 빌드해둔 ROS2+SSH 기반 이미지를 사용
FROM ros:humble-ssh

# 작업 디렉토리 설정
WORKDIR /home/ubuntu/ros2_ws

# 필요한 파이썬 패키지 설치
COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir -r requirements.txt

# 소스 코드 복사
COPY ./src ./src

# ROS2 환경 변수 설정 (zsh 사용 시 .zshrc에 추가)
RUN echo "source /opt/ros/humble/setup.zsh" >> /home/ubuntu/.zshrc
RUN echo "export PYTHONUNBUFFERED=1" >> /home/ubuntu/.zshrc # 파이썬 로그가 바로 보이도록 설정

# 컨테이너 시작 시 실행될 기본 명령어
# 여기서는 sshd를 그대로 사용하고, 사용자가 접속해서 직접 노드를 실행하는 시나리오
# 만약 컨테이너 시작 시 자동으로 노드가 실행되게 하려면 CMD를 변경
# CMD ["ros2", "run", "my_package_name", "main_node.py"]
CMD ["/usr/sbin/sshd", "-D"]
