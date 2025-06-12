#!/bin/bash

# ===================================================================
# GitHub Actions를 통해 호출되는 자동 배포 스크립트 (test.py 용)
# ===================================================================

set -e

# --- 1. 프로젝트 디렉토리로 이동 ---
PROJECT_DIR=$(dirname "$0")
cd "$PROJECT_DIR"
echo "✅ Picar> Working directory set to: $(pwd)"

# --- 2. 최신 코드 가져오기 ---
echo "🔄 Picar> Pulling the latest code from origin/main..."
git pull origin sh-dev

# --- 3. 기존 테스트 스크립트 종료 ---
# 이전에 실행되던 test.py가 있다면 먼저 종료합니다.
echo "🔄 Picar> Attempting to stop existing test.py process..."
pkill -f "python3 src/test.py" || true
sleep 1

# --- 4. 새로운 테스트 스크립트 실행 ---
# 'src' 폴더 안에 있는 test.py를 파이썬으로 직접 실행합니다.
# nohup과 &를 사용하여 백그라운드에서 계속 실행되도록 합니다.
echo "🚀 Picar> Launching src/test.py in the background..."
nohup python3 src/test.py &

echo "✅ Picar> Deployment finished successfully! test.py is running."

