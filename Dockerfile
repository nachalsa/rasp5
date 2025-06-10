FROM hwangsanha7/rasp5-base:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    zsh \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN mkdir src
