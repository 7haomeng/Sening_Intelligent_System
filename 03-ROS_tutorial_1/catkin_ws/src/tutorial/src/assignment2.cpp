#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Matrix3x3 R1(),//***********************************************************//
              R2(),//                                                           //
              R3(),//  What parameters should be added in the constructor?      //
              R4(),//                                                           //
              R5();//***********************************************************//
tf::Vector3 p1(),//***********************************************************//
            p2(),//                                                           //
            p3(),//  What parameters should be added in the constructor?      //
            p4(),//                                                           //
            p5();//***********************************************************//


void broadcastTF(const ros::TimerEvent& timer_event){
  // T1: B -> A
  // T2: A -> C
  // T3: E -> D
  // T4: C -> D
  // T5: A -> F
  tf::Transform T1, T2, T3, T4, T5;
  tf::Transform dummy_tf; // Not use actually, just to make the file compilable
  tf::Quaternion q;
  // After you correct the constructors, uncomment here
  /*T1.setOrigin(p1); R1.getRotation(q); T1.setRotation(q);
  T2.setOrigin(p2); R2.getRotation(q); T2.setRotation(q);
  T3.setOrigin(p3); R3.getRotation(q); T3.setRotation(q);
  T4.setOrigin(p4); R4.getRotation(q); T4.setRotation(q);
  T5.setOrigin(p5); R5.getRotation(q); T5.setRotation(q); */
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(dummy_tf// Which transform should be used here?
                                        , ros::Time::now(), "A", "B"));
  br.sendTransform(tf::StampedTransform(dummy_tf// Which transform should be used here?
                                        , ros::Time::now(), "B", "C"));
  br.sendTransform(tf::StampedTransform(dummy_tf// What transform should be used here?
                                        , ros::Time::now(), "C", "D"));
  br.sendTransform(tf::StampedTransform(dummy_tf// Which transform should be used here?
                                        , ros::Time::now(), "C", "E"));
  br.sendTransform(tf::StampedTransform(dummy_tf// Which transform should be used here? 
                                        , ros::Time::now(), "E", "F"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "assignment2_node");
  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()) {ros::spinOnce();} 
  return 0;
}
