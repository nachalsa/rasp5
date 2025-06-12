#!/bin/bash

# 에러 발생 시 스크립트 중지
set -e

echo ">> 1. 최신 소스 코드를 main 브랜치에서 가져옵니다..."
git fetch origin main
git checkout main
git pull origin main

echo ">> 2. Docker Compose를 사용하여 기존 컨테이너를 중지하고 삭제합니다..."
# 컨테이너가 실행 중이지 않아도 오류가 발생하지 않도록 `|| true` 추가
docker-compose down || true

echo ">> 3. 새로운 코드로 Docker 이미지를 다시 빌드합니다..."
docker-compose build

echo ">> 4. 새로운 이미지로 컨테이너를 백그라운드에서 시작합니다..."
docker-compose up -d

