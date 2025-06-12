# 파일 경로: /Dockerfile (프로젝트 루트)

# 공식 ROS2 humble 베이스 이미지에서 시작합니다.
FROM ros:humble-ros-base

# Docker 빌드 시 실행될 셸을 bash로 설정합니다.
SHELL ["/bin/bash", "-c"]

# [★가장 중요★] apt 패키지 리스트를 업데이트하고, colcon 확장 기능과 함께
# 모든 린팅 도구를 한번에 설치해주는 'ament-lint-auto'를 명시적으로 설치합니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-ament-lint-auto \
    && rm -rf /var/lib/apt/lists/*

# CI에서 사용할 작업공간을 미리 생성합니다.
RUN mkdir -p /ros2_ws/src

# 이 컨테이너의 기본 작업 디렉토리를 설정합니다.
WORKDIR /ros2_ws
