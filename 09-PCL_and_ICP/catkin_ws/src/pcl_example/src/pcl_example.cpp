#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
  pcl::fromROSMsg (*input, *cloud); //convert from PointCloud2 to pcl point type
  //Exmaple : pcl PointCloudXYZRGB information
  printf("-------------------------Cloud information-----------------------------\n");
  printf("Original Cloud size: %d\n",cloud->points.size());
  int cloud_size=cloud->points.size();
  printf("The first cloud coordinate and color information:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf, R: %d, G: %d, B: %d\n",cloud->points[0].x,cloud->points[0].y,cloud->points[0].z,cloud->points[0].r,cloud->points[0].g,cloud->points[0].b);
  printf("The last cloud coordinate and color information:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf, R: %d, G: %d, B: %d\n",cloud->points[cloud_size-1].x,cloud->points[cloud_size-1].y,cloud->points[cloud_size-1].z,cloud->points[cloud_size-1].r,cloud->points[cloud_size-1].g,cloud->points[cloud_size-1].b);


  std::vector<int> indices;
  PointCloudXYZRGB::Ptr filtered_cloud(new PointCloudXYZRGB);
  pcl::removeNaNFromPointCloud(*cloud,*filtered_cloud, indices);
  printf("Nonnan Cloud Number: %d\n",filtered_cloud->points.size());
  printf("**********************************************************************\n");

}
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

     // Spin
     ros::spin ();
  }
