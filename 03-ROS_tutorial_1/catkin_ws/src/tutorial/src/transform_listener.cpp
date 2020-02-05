#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_listener_node");
  ros::NodeHandle nh;
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  
  std::cout << "Wait for 1 seconds..." << std::endl;
  ros::Duration(1.0).sleep();

  try{
    listener.lookupTransform("A", // target_frame
                             "C", // source_frame
                             ros::Time(0),
                             transform);
    }
  catch (tf::TransformException ex){
    //ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // Convert quaternion to RPY
  tf::Quaternion q(transform.getRotation().x(),
                   transform.getRotation().y(),
                   transform.getRotation().z(),
                   transform.getRotation().w());
  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  std::cout.setf(std::ios::fixed);
  std::cout << "Translation: " << std::endl
            << "  x: " << std::setprecision(3) << transform.getOrigin().x()   << std::endl
	    << "  y: " << std::setprecision(3) << transform.getOrigin().y()   << std::endl
            << "  z: " << std::setprecision(3) << transform.getOrigin().z()   << std::endl
            << "Rotation (in quaternion):"                                    << std::endl
            << "  x: " << std::setprecision(3) << transform.getRotation().x() << std::endl
            << "  y: " << std::setprecision(3) << transform.getRotation().y() << std::endl
            << "  z: " << std::setprecision(3) << transform.getRotation().z() << std::endl
            << "  w: " << std::setprecision(3) << transform.getRotation().w() << std::endl
            << "Rotation (in RPY, radians): "                                 << std::endl
            << "  R: " << std::setprecision(3) << roll                        << std::endl
            << "  P: " << std::setprecision(3) << pitch                       << std::endl
            << "  Y: " << std::setprecision(3) << yaw                         << std::endl
            << "Rotation (in RPY, degrees): "                                 << std::endl
            << "  R: " << std::setprecision(3) << (roll/M_PI) * 180.0         << std::endl
            << "  P: " << std::setprecision(3) << (pitch/M_PI) * 180.0        << std::endl
            << "  Y: " << std::setprecision(3) << (yaw/M_PI) * 180.0          << std::endl;
  return 0;
}
