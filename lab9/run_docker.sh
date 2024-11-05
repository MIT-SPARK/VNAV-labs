#!/bin/bash
DISPLAY_NUM=$(echo $DISPLAY | cut -c 2 -)
# https://stackoverflow.com/questions/59895/how-do-i-get-the-directory-where-a-bash-script-is-located-from-within-the-script
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

xhost +si:localuser:root
sudo docker run \
    -v /tmp/.X11-unix/X$DISPLAY_NUM:/tmp/.X11-unix/X$DISPLAY_NUM \
    -v $HOME/datasets/vnav:/datasets:ro \
    -v $SCRIPT_DIR/output:/output:rw \
    -v $SCRIPT_DIR/params:/kimera_params:ro \
    -e DISPLAY \
    --net=host \
    -it $1
xhost -si:localuser:root
