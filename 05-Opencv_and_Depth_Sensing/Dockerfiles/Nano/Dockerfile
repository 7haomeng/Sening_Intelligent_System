
From argnctu/sis_lab5
RUN mkdir -p /home/nvidia/sis_lab_all/
WORKDIR /home/nvidia/sis_lab_all/
COPY . /home/nvidia/sis_lab_all/05-Opencv_and_Depth_Sensing

WORKDIR /home/nvidia/sis_lab_all/05-Opencv_and_Depth_Sensing/catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
