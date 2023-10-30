#!/bin/bash
DISPLAY_NUM=$(echo $DISPLAY | cut -c 2 -)

xhost +si:localuser:root
sudo docker run \
    -v /tmp/.X11-unix/X$DISPLAY_NUM:/tmp/.X11-unix/X$DISPLAY_NUM \
    -v $HOME/datasets/vnav:/datasets \
    -e DISPLAY \
    --net=host \
    -it $1
xhost -si:localuser:root
