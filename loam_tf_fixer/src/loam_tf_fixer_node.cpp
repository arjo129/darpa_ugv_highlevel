#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> out;

void callback(const PointCloud::ConstPtr& msg)
{
  geometry_msgs::Transform trans;
  trans.translation.x = 0;
  trans.translation.y = 0;
  trans.translation.z = 0;
  trans.rotation.x = 0.5;
  trans.rotation.y = 0.5;
  trans.rotation.z = 0.5;
  trans.rotation.w = 0.5;
  pcl_ros::transformPointCloud	(*msg , out , trans);
  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2 (out ,pc2);
  pub.publish(pc2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("X1/points", 1, callback);
  pub = nh.advertise<PointCloud> ("X1/points2", 1);
  ros::spin();
}


