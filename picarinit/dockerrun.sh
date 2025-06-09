docker run -dit \
--name humble \
--privileged \
--restart always \
--network host \
-e DISPLAY=$DISPLAY \
-v /dev:/dev \
-v $HOME/docker/tmp:/home/ubuntu/shared \
ros:humble-ssh
