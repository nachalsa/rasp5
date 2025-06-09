#!/bin/bash
#in raspi
# 룰 추가
echo "룰을 추가합니다..."
sudo cp ~/picarinit/rules/*.rules /etc/udev/rules.d/.
sudo mv ~/picarinit/setup_wipicar_hotspot.ch /usr/local/bin/setup_wipicar_hotspot.ch
sudo mv ~/picarinit/wipicar-hotspot.service /etc/systemd/system/wipicar-hotspot.service
sudo systemctl daemon-reload

echo "룰 추가 완료."

# 시스템 업데이트 및 업그레이드
echo "시스템 업데이트 및 업그레이드를 시작합니다..."
sudo apt-get update -y
sudo apt-get upgrade -y
echo "시스템 업데이트 및 업그레이드 완료."

# Vim 설치
echo "Vim을 설치합니다..."
sudo apt-get install vim -y
echo "Vim 설치 완료."

# .bashrc 설정 변경
echo "~/.bashrc 설정을 변경합니다..."
echo "alias ll='ls -alF'" >> ~/.bashrc
echo "xhost +local:docker > /dev/null 2>&1" >> ~/.bashrc

# 히스토리 크기 설정 (기존 값 변경 또는 추가)
sed -i '/^HISTSIZE=/c\HISTSIZE=100000' ~/.bashrc || echo 'HISTSIZE=100000' >> ~/.bashrc
sed -i '/^HISTFILESIZE=/c\HISTFILESIZE=200000' ~/.bashrc || echo 'HISTFILESIZE=200000' >> ~/.bashrc
echo "~/.bashrc 설정 변경 완료. 재부팅 후 적용됩니다."

# 호스트 이름 변경
echo "호스트 이름을 'raspberrypi'에서 'picar'로 변경합니다..."
sudo sed -i '/^raspberrypi/c\picar' /etc/hostname
sudo sed -i '/^127.0.1.1		raspberrypi/c\127.0.0.1		picar' /etc/hosts
echo "호스트 이름 변경 완료. 재부팅 후 적용됩니다."

echo "Docker를 설치합니다..."

# Docker 패키지 제거 (혹시 이전에 설치된 것이 있다면)
sudo apt-get remove docker docker-engine docker.io containerd runc -y

# Docker 저장소 설정
# Add Docker's official GPG key:
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/raspbian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/raspbian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Docker 엔진 설치
sudo apt-get update -y
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
echo "Docker 설치 완료."

# 현재 사용자에게 Docker 그룹 권한 추가
echo "현재 사용자 ($USER)를 docker 그룹에 추가합니다..."
sudo usermod -aG docker $USER
echo "사용자 그룹 변경 완료. 재부팅 후 적용됩니다."

# 시스템 재부팅
echo "모든 변경 사항 적용을 위해 시스템을 재부팅합니다..."
sudo reboot
