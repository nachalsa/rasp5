# Dockerfile

# 공식 ROS2 humble 베이스 이미지에서 시작합니다.
FROM ros:humble-ros-base

# Docker 빌드 시 실행될 셸을 bash로 설정합니다.
SHELL ["/bin/bash", "-c"]

# apt 패키지 리스트를 업데이트하고, 린팅에 필요한 핵심 도구들을 설치합니다.
# - python3-colcon-common-extensions: colcon의 확장 기능들을 제공합니다.
# - ros-humble-ament-lint-auto: flake8, pep257 등 필요한 모든 린터들을 한번에 설치해주는 패키지입니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ament-lint-auto \
    && rm -rf /var/lib/apt/lists/*

# CI/CD에서 사용할 작업공간을 미리 생성합니다.
RUN mkdir -p /ros2_ws/src

# 이 컨테이너의 기본 작업 디렉토리를 설정합니다.
WORKDIR /ros2_ws

