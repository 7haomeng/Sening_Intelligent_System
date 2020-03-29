xhost +local:docker
docker run --runtime nvidia -it --rm --name lab5 --net host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --privileged -v /dev:/dev  -w  /home/nvidia/sis_lab_all/05-Opencv_and_Depth_Sensing/catkin_ws/ lab5 bash

