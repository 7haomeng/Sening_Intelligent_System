#!/usr/bin/env sh

# Setup the style of color
RED='\033[0;31m'
NC='\033[0m'

# Find current directory and transfer it to container directory for Docker
current_dir="$(pwd)"
host_dir="/home/$USER/"
container_dir="/hosthome/"
goal_dir=${current_dir//$host_dir/$container_dir}
echo ${goal_dir}

docker run -it --rm \
    -v /home/$USER:/hosthome \
    --net host \
    --runtime nvidia \
    -w ${goal_dir} \
    argnctu/pytorch-jupyter

