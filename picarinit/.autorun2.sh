#!/bin/bash
#in raspi
docker image load -i ~/picarinit/ros-humble-export.tar
sudo mkdir -p ~/docker/humble
sudo chown $USER:$USER ~/docker/humble
echo 'FROM ros:humble-export
RUN apt-get update && apt-get install -y openssh-server \
    && mkdir -p /var/run/sshd

RUN sed -i "s/#Port 22/Port 2222/" /etc/ssh/sshd_config
RUN echo "ubuntu:12345678" | chpasswd
RUN chsh -s /usr/bin/zsh ubuntu

EXPOSE 2222

CMD ["/usr/sbin/sshd", "-D"]' > ~/docker/humble/dockerfile
docker build -t ros:humble-ssh ~/docker/humble/.
docker run -dit \
--name humble \
--privileged \
--restart always \
--network host \
-e DISPLAY=$DISPLAY \
-v /dev:/dev \
-v $HOME/docker/tmp:/home/ubuntu/shared \
ros:humble-ssh
