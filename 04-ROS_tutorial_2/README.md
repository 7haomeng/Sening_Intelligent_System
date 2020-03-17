## 04-ROS_tutorial_2 filesystem
*  catkin_ws/: workspace
    *  src/: packages inside
        *  apriltag_ros/: apriltag ros package, cloned from https://github.com/RIVeR-Lab/apriltags_ros  
            *  What we need to do is make a copy of apriltag_ros/launch/example.launch and revise parameters
    *  tutorial/: student tutorial package
        *  src/:
            *  assignment.cpp: for assignment, semi finished, students have to implement by themselves
            *  client_call.cpp: call service from node, Topic 1
            *  service_server.cpp: define the behavior of service, Topic 1 
            *  tag_distance.py: measure the distance between the tags, Topic 2.2
        *  srv/:
            *  assignment.srv: for assignment, finished, students don't have to implement
            *  my_service.srv: Topic 1 
        *  config/:
            *  rviz/: rviz config file
            *  haarcascade_frontalface_alt.xml: OpenCV haar cascade face feature config file
        *  image/: result images will be saved here
        *  launch/:
            *  assignment.launch: for assignment, semi finished, students have to implement by themselves
            *  face_detection_server.launch: Topic 2.1
            *  tag_distance.launch: Topic 2.2
            *  topic2.launch: Topic 2.2
    *  usb_cam: usb_cam package from ros_driver, cloned from https://github.com/ros-drivers/usb_cam  
*  Dockerfile: sean85914/04-ros-tutorial-2 image docker file
