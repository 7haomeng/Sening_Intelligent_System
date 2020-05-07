xhost +local:docker
docker run --runtime nvidia -it --rm --name lab5 --net host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged -v /dev:/dev \
	-v /home/$USER/sis_lab_all_2020/09-PCL_and_ICP:/home/nvidia/sis_lab_all/09-PCL_and_ICP \
       	-w  /home/nvidia/sis_lab_all/ lab5 bash

