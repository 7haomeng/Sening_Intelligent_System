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

# Check the command 'nvidia-docker' is existing or not
ret_code="$(command -v nvidia-docker)"
if [ -z "$ret_code" ]
then
    printf "${RED}\"nvidia-docker\" is not found, so substitute docker. $NC\n"
    docker run -it --rm -v /home/$USER:/hosthome -p 8888:8888 -w ${goal_dir} pytorch-jupyter 
else
    printf "Run \"nvidia-docker\"\n"
    nvidia-docker run -it --rm -v /home/$USER:/hosthome -p 8888:8888 -w ${goal_dir} pytorch-jupyter 
fi
