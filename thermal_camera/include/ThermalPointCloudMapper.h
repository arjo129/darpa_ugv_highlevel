#ifndef THERMAL_POINT_CLOUD_MAPPER_SL
#define THERMAL_POINT_CLOUD_MAPPER_SL

#include <chrono>
#include <vector>
#include <math.h>
#include <string>
#include <iostream>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#define TF_THERMAL_FRAME "thermal_optical_frame"

using namespace std;

struct Affine {
    double Rxx, Rxy, Rxz, Ryx, Ryy, Ryz, Rzx, Rzy, Rzz;
    double tx, ty, tz;
};

class ThermalPointCloudMapper {
    ros::NodeHandle _nh;

    ros::Subscriber sub_thermal_img, sub_depth_img;
    ros::Publisher _filteredPoints_visual, _actual_visual;
    image_transport::Publisher pub_thermal_image;

    Affine frameTransform;

    sensor_msgs::ImageConstPtr depthImgPtr;

    void setBgrToArray(uchar* arr, uchar blue, uchar green, uchar red);

    bool isAcceptableRange(int z);
    bool isBounded(int val, int lower, int upper);
    void depth_img_callback(const sensor_msgs::ImageConstPtr& msg);
    void thermal_img_callback(const sensor_msgs::ImageConstPtr& msg);

    int getWatershedMarkers(const cv::Mat &src, cv::Mat &markers);
    Affine getAffinityMatrix(const geometry_msgs::Transform &tf);
    uchar map(uchar val, uchar lowerbound, uchar upperbound);

    cv::Point2i project3dToPixel(const cv::Point3d &pt, double HFOV, double VFOV, int img_h, int img_w);
    cv::Point3d pixelToProject3d(int x, int y, double depth, double HFOV, double VFOV, int img_h, int img_w);
    cv::Point3d transform(cv::Point3d &pt, const Affine *tf);

    cv::Mat getDirectImageMap(const cv::Mat &depth_img, const cv::Mat &ref_img,
                                double depth_HFOV, double depth_VFOV,
                                double ref_HFOV, double ref_VFOV,
                                const Affine *sourceToRefTransform);
    cv::Mat getSmartThermalOverlay(const cv::Mat &depth_img, const cv::Mat &mapped_thermal_img);


public:
    ThermalPointCloudMapper();
};

constexpr double toRad(double degs) {
    return degs * (M_PI / 180.0);
}
#endif