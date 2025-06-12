# 파일 경로: /Dockerfile

FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# ★★★ 여기가 핵심! 린트 도구를 설치하는 이 부분이 반드시 있어야 합니다. ★★★
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-ament-lint-auto \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
