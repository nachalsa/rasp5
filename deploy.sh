#!/bin/bash

# ===================================================================
# GitHub Actions를 통해 호출되는 자동 배포 스크립트
#
# 역할: 최신 코드를 받고, 프로젝트를 빌드한 후, ROS2 프로그램을 실행
# 위치: /home/car/rasp5/deploy.sh
# ===================================================================

# --- 스크립트 기본 설정 ---
# 한 명령어라도 실패하면 즉시 스크립트 실행을 중단합니다. (매우 중요)
set -e

# --- 사용자 설정 변수 (이곳만 수정하세요!) ---
ROS_DISTRO="humble"
PACKAGE_NAME="rasp5"
LAUNCH_FILE="cicd.test.py"

# --- 1. 프로젝트 디렉토리로 이동 ---
# 이 스크립트 파일이 위치한 디렉토리로 안전하게 이동합니다.
# 이렇게 하면 항상 올바른 위치에서 작업이 수행됩니다.
PROJECT_DIR=$(dirname "$0")
cd "$PROJECT_DIR"
echo "✅ Picar> Working directory set to: $(pwd)"


# --- 2. 기존 ROS2 프로세스 종료 ---
# 이전 버전의 프로그램이 실행 중일 수 있으므로, 먼저 종료합니다.
# 'pkill'은 프로세스 이름으로 찾아서 종료하며, '-f' 옵션은 전체 명령어 라인을 검색합니다.
# '|| true'는 종료할 프로세스가 없어서 명령이 실패해도 스크립트가 중단되지 않도록 합니다.
echo "🔄 Picar> Attempting to stop existing ROS2 launch process..."
pkill -f "ros2 launch $PACKAGE_NAME $LAUNCH_FILE" || true
# 프로세스가 완전히 종료될 시간을 벌어줍니다.
sleep 2


# --- 3. 최신 코드 가져오기 ---
# 원격(GitHub) main 브랜치의 최신 변경사항을 로컬로 가져옵니다.
echo "🔄 Picar> Pulling the latest code from origin/main..."
git pull origin main


# --- 4. ROS2 환경 설정 ---
# ROS2 환경과 빌드된 우리 프로젝트의 환경을 로드합니다.
echo "🔄 Picar> Sourcing ROS2 environment..."
# ROS2 기본 환경 로드
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "❌ Error: ROS2 environment file not found at /opt/ros/$ROS_DISTRO/setup.bash"
    exit 1
fi
# 로컬 프로젝트 환경 로드
if [ -f "./install/setup.bash" ]; then
    source "./install/setup.bash"
else
    echo "⚠️ Warning: Local setup file not found. Proceeding with colcon build."
fi


# --- 5. 프로젝트 빌드 ---
# 새로 받은 코드를 적용하기 위해 colcon으로 프로젝트를 다시 빌드합니다.
echo "🔄 Picar> Building the project with colcon..."
colcon build


# --- 6. 새로 빌드된 환경 다시 로드 ---
# 빌드를 통해 생성/수정된 install/setup.bash를 다시 로드하여 최신 상태를 반영합니다.
echo "🔄 Picar> Re-sourcing the local environment..."
source ./install/setup.bash


# --- 7. 새로운 프로그램 실행 ---
# 최신 버전의 launch 파일을 백그라운드(&)에서 실행합니다.
# '&'를 붙여야 이 스크립트가 종료된 후에도 프로그램이 계속 실행됩니다.
# 'nohup'은 터미널 세션이 끊겨도 프로세스가 계속 실행되도록 보장합니다. (더 안정적)
echo "🚀 Picar> Launching the new application in the background..."
nohup ros2 launch "$PACKAGE_NAME" "$LAUNCH_FILE" &


echo "✅ Picar> Deployment finished successfully! The application is running."
