#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event){
  static tf::TransformBroadcaster br;
  tf::Transform tf_A_B, tf_B_C;
  tf::Quaternion q;
  tf_A_B.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
  tf_B_C.setOrigin(tf::Vector3(0.0, 0.5, 0.5));
  q.setRPY(M_PI/2, M_PI/2, 0);
  tf_A_B.setRotation(q);
  q.setRPY(0, 0, M_PI);
  tf_B_C.setRotation(q);
  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform 
					"A", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_B_C, 
                                        ros::Time::now(), 
					"B", 
                                        "C"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_transform_node");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}  
  return 0;
}
