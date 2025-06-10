 =================================================================
# Dockerfile  for Continuous Integration (CI) - 최종 완성본
# 
# FROM: Docker Hub에 업로드한 실제 이미지 주소를 정확히 기입합니다.
# RUN: 로봇의 설정 스크립트에서 가져온 필수 패키지들을 설치합니다.
# WORKDIR/RUN: ROS2 프로젝트를 빌드하고 테스트할 준비를 합니다.
# =================================================================

# 1. 베이스 이미지 지정
FROM hwangsanha7/rasp5-base:humble

# 2. 추가 프로그램 설치
# 로봇 설정 스크립트에 있던 vim, zsh 등을 동일하게 설치합니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    zsh \
    && rm -rf /var/lib/apt/lists/*

# 3. 작업 공간(디렉토리) 설정
# 컨테이너 안에서 ROS 작업을 할 기본 폴더를 /ros2_ws로 지정합니다.
WORKDIR /ros2_ws

# 4. 소스 코드 복사 준비
# 나중에 CI가 우리 GitHub 프로젝트의 'src' 폴더 내용을 이곳으로 가져올 수 있도록
# 'src' 폴더를 미리 만들어 둡니다.
RUN mkdir src
