#!/bin/bash
docker run \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --volume="$XAUTHORITY:/root/.Xauthority:rw" \
    --volume=$(pwd)/catkin_ws:/root/catkin_ws \
    --rm \
    --name brtc \
    -it br-tech-challenge 
