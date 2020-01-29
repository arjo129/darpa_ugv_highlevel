#ifndef THERMAL_POINT_CLOUD_MAPPER_SL
#define THERMAL_POINT_CLOUD_MAPPER_SL

#include <vector>
#include <math.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <opencv/cv.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class ThermalPointCloudMapper {
    ros::NodeHandle _nh;

    ros::Subscriber sub_cloud, sub_image;
    ros::Publisher _filteredPoints_visual, _actual_visual;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw;

    bool isBounded(int val, int lower, int upper);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void img_callback(const sensor_msgs::ImageConstPtr& msg);
    cv::Point2d project3dToPixel(double x, double y, double z, int img_w, int img_h);

public:
    ThermalPointCloudMapper();

};
#endif