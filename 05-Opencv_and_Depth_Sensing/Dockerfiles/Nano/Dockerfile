
From peterx7803/ctsp-00-ros
RUN mkdir -p /home/nvidia/sis_lab_all_2020/
WORKDIR /home/nvidia/sis_lab_all_2020/
COPY . /home/nvidia/sis_lab_all_2020/05-Opencv_and_Depth_Sensing

WORKDIR /home/nvidia/sis_lab_all_2020/05-Opencv_and_Depth_Sensing/catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
