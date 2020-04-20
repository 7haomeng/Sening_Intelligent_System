#!/usr/bin/env bash
xhost +local:docker

# New docker user
DOCKER_USER="developer"

COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[0;33m'
COLOR_NC='\033[0m'

#
# Check the command 'nvidia-docker' existed or not
#
ret_code="$(command -v nvidia-docker)"
if [ -z "$ret_code" ]; then
    DOCKER_CMD="docker"
else
    DOCKER_CMD="nvidia-docker"
fi


#
# Specify cuda version
#
if [ $# -gt 0 ]; then
    if [[ "$1" == "cuda9" || "$1" == "cuda9.0" ]] ; then
        echo -e "RUN: \"${DOCKER_CMD}\""
        DOCKER_TAG="latest"
    elif [[ "$1" == "cuda10" || "$1" == "cuda10.0" ]] ; then
        echo -e "RUN: \"${DOCKER_CMD}\""
        DOCKER_TAG="latest"
    elif [ "$1" == "same" ] ; then
        echo -e "RUN: \"docker exec\""
    else
        echo -e "Please specify which cuda version your GPU support."
        echo -e "${COLOR_RED}Usage: source docker_run.sh [cuda9 | cuda10 | same]${COLOR_NC}"
    fi
else
    echo -e "${COLOR_RED}Usage: source docker_run.sh [cuda9 | cuda10| same]${COLOR_NC}"
fi



#
# Execute command
#
if [ $# -gt 0 ]; then
    if [ "$1" == "same" ]; then
        docker exec -it pyrobot bash
    else
        ${DOCKER_CMD} run --name pyrobot --rm -it --network=host --privileged \
            -v /dev:/dev \
            -e DISPLAY=$DISPLAY \
            -v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
            -v /home/$USER/sis_lab_all_2020:/home/${DOCKER_USER}/sis_lab_all_2020 \
            -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
            -w /home/${DOCKER_USER}/sis_lab_all_2020 \
            --device=/dev/dri \
            --device=/dev/nvhost-ctrl \
            --device=/dev/nvhost-ctrl-gpu \
            --device=/dev/nvhost-prof-gpu \
            --device=/dev/nvmap \
            --device=/dev/nvhost-gpu \
            --device=/dev/nvhost-as-gpu \
            -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
            -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
            --ipc host \
            argnctu/sis_lab7:${DOCKER_TAG} 
    fi
else
    echo "please provide docker tag name."
fi
