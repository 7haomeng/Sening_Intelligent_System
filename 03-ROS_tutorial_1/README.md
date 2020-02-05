# Sensing and Intelligent System, 2018, lab3, ROS tutorial(I)

### File system
* catkin_ws: workspace
  * src: packages inside
    * sensor: package related to the sensors
      * script
        * barometric: Arduino project folder
      * src: nodes inside
        * read_data.py: read data from Arduino and publish as ROS topics
      * CMakeLists.txt
      * package.xml
    * tutorial: package related to the in-class tutorial
      * src: nodes inside
        * assignment1.py: for usage of assignment 1, students have to finish this by themselves
        * assignment2.py: for usage of assignment 2, students have to finish this by themselves
        * broadcast_transform.cpp: for usage of Topic 3, already finished
        * convert_temp_unit.py: for usage of Topic 2, already finished
        * test_my_message.py: for usage of Topic 1, already finished
        * transform_listener.cpp: for usage of topic 3, already finished
      * msg
        *  my_message.msg: Self-defined new type message, for usage of Topic 2
      * CMakeLists.txt
      * package.xml
* rviz: rviz config file placeholder
