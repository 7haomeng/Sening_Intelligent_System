#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher marker_pub;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
	pcl::fromROSMsg (*input, *cloud); //convert from PointCloud2 to pcl point type
	std::vector<int> indices;
	PointCloudXYZRGB::Ptr filtered_cloud(new PointCloudXYZRGB);
	pcl::removeNaNFromPointCloud(*cloud,*filtered_cloud, indices);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

	// Set the input pointcloud for the search tree
	tree->setInputCloud (filtered_cloud);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
  	ne.setInputCloud (filtered_cloud);
  	ne.setSearchMethod (tree);

  	/**
   	* NOTE: setting viewpoint is very important, so that we can ensure
   	* normals are all pointed in the same direction!
   	*/
  	ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

	// Calculate normals
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
	ne.setRadiusSearch (0.01);
	ne.compute (*normals);

	// Visualize line lists
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "camera_color_optical_frame";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 2;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	//line_list.scale.x = 0.001;
	line_list.scale.x = 0.001;
	line_list.color.r = 1.0;
  	line_list.color.a = 1.0;
	printf  ("%d\n",normals->points.size());
	printf  ("%.4lf %.4lf %.4lf\n",normals->points[0].normal_x,normals->points[0].normal_y,normals->points[0].normal_z);
	for (int i=0; i<filtered_cloud->points.size();i++){
		if (i%300 == 0 && std::isnan(normals->points[i].normal_x) == false){
			geometry_msgs::Point p1;
			p1.x = filtered_cloud->points[i].x;
			p1.y = filtered_cloud->points[i].y;
			p1.z = filtered_cloud->points[i].z;
			line_list.points.push_back(p1);
			geometry_msgs::Point p2 = p1;
			p2.x = p2.x + normals->points[i].normal_x/10.0;
			p2.y = p2.y + normals->points[i].normal_y/10.0;
			p2.z = p2.z + normals->points[i].normal_z/10.0;
			line_list.points.push_back(p2);
		}

	}
	marker_pub.publish(line_list);
}
int   main (int argc, char** argv)
{
     	// Initialize ROS
     	ros::init (argc, argv, "my_pcl_tutorial");
     	ros::NodeHandle nh;
     	// Create a ROS subscriber for the input point cloud
     	ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
     	// Spin
     	ros::spin ();
  }
