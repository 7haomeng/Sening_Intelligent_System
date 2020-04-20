#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Matrix3x3 R1(0.0, -1.0, 0.0,
		             1.0, 0.0, 0.0,
		             0.0, 0.0, 1.0),//***********************************************************//
              R2(0.0, 1.0, 0.0, 		
		             0.0, 0.0, -1.0,
		             -1.0, 0.0, 0.0),//                                                           //
              R3(0.7500, -0.2165, 0.6250,
		             0.4330, 0.8750, -0.2165,
		             -0.5000, 0.4330, 0.7500),//  What parameters should be added in the constructor?      //
              R4(0.0000, -0.8660, 0.5000,
		             0.5000, 0.4330, 0.7500,
		             -0.8660, 0.2500, 0.4330),//                                                           //
              R5(0.5000, -0.1464, 0.8536,
		             0.5000, 0.8536, -0.1464,
		             -0.7071, 0.5000, 0.5000);//***********************************************************//
tf::Vector3 p1(-0.3, 0.4, 0.5),//***********************************************************//
            p2(0.6, 0.8, 0.0),//                                                           //
            p3(0.3, 0.3, 0.3),//  What parameters should be added in the constructor?      //
            p4(0.6, 0.4, 0.0),//                                                           //
            p5(0.1, 0.2, 0.3);//***********************************************************//


void broadcastTF(const ros::TimerEvent& timer_event){
  tf::Transform T1, T2, T3, T4, T5;
  
  tf::Transform dummy_tf; // Not use actually, just to make the file compilable
  tf::Quaternion q;
  // After you correct the constructors, uncomment here
  T1.setOrigin(p1); R1.getRotation(q); T1.setRotation(q);
  T2.setOrigin(p2); R2.getRotation(q); T2.setRotation(q);
  T3.setOrigin(p3); R3.getRotation(q); T3.setRotation(q);
  T4.setOrigin(p4); R4.getRotation(q); T4.setRotation(q);
  T5.setOrigin(p5); R5.getRotation(q); T5.setRotation(q);
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(T1.inverse()//dummy_tf// Which transform should be used here?
                                        , ros::Time::now(), "A", "B"));
  br.sendTransform(tf::StampedTransform(T1 * T2//dummy_tf// Which transform should be used here?
                                        , ros::Time::now(), "B", "C"));
  br.sendTransform(tf::StampedTransform(T4//dummy_tf// What transform should be used here?
                                        , ros::Time::now(), "C", "D"));
  br.sendTransform(tf::StampedTransform(T4 * T3.inverse()//dummy_tf// Which transform should be used here?
                                        , ros::Time::now(), "C", "E"));
  br.sendTransform(tf::StampedTransform(T3 * T4.inverse() * T2.inverse() * T5//dummy_tf// Which transform should be used here? 
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
