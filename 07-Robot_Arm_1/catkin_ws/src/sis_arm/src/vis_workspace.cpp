#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <cmath>

class FK 
{
  private:
    const double _d0 = 0.045, _d1 = 0.06, _a2 = 0.065, _d3 = 0.07;
    double x_, y_, z_; // Position
    double j1_, j2_; // Joint values
    double c1_, s1_, c2_, s2_;
    double j1_ulimit_ = 2.616, j1_llimit_ = -2.616, j2_ulimit_ = 0.8, j2_llimit_ = -2.0;
    int count_ = 0; // count points number
    ros::NodeHandle nh_; 
    ros::Publisher pub_marker_;
    visualization_msgs::Marker marker_;
    void make_marker(double x, double y, double z) {  
      marker_.header.frame_id = "base";
      marker_.type = visualization_msgs::Marker::POINTS;
      marker_.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point p;
      p.x = x; p.y = y; p.z = z;
      std_msgs::ColorRGBA color;
      color.r = 0; color.g = 1.0; color.b = 0; color.a = 1.0;
      marker_.scale.x = marker_.scale.y = marker_.scale.z = 0.001;
      marker_.points.push_back(p); marker_.colors.push_back(color);
    }
  public:
    FK() {
      pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/workspace", 10);
      for (j1_ = j1_llimit_; j1_ <= j1_ulimit_; j1_ += 0.03) {
        for (j2_ = j2_llimit_; j2_ <= j2_ulimit_; j2_ += 0.03) {
          c1_ = cos(j1_), s1_ = sin(j1_), c2_ = cos(j2_), s2_ = sin(j2_);
          x_ = -_a2 * s1_ * s2_ - _d3 * s1_ * c2_;
          y_ = _d1 + _a2 * c2_ - _d3 * s2_;
          z_ = _d0 - _a2 * c1_ * s2_ - _d3 * c1_ * c2_;
          make_marker(x_, y_, z_);
        }
        ++count_;
        if(count_%10 == 0) {ROS_INFO("There are %d points now.", count_);}
      }
      ROS_INFO("Marker initial comppleted, start publish marker...");
    }
    ~FK() {}
    void publish_marker() {
      pub_marker_.publish(marker_);
    }
}; 

int main(int argv, char** argc)
{
  ros::init(argv, argc, "visualize_workspace_node");
  FK fk;
  ros::Rate r(1); // 1Hz
  while(ros::ok()) {
    fk.publish_marker();
     r.sleep();
  }
  return 0;
}
