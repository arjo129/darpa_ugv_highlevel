#include "../include/ThermalPointCloudMapper.h"

const double FOV = 110.0 * (M_PI / 180.0);
const double THERMAL_THRESH = 30;

ThermalPointCloudMapper::ThermalPointCloudMapper() : cloud_raw(new pcl::PointCloud<pcl::PointXYZ>) {
    // define the subscriber and publisher
    sub_cloud = _nh.subscribe("/camera/depth/color/points", 1, &ThermalPointCloudMapper::cloud_cb, this);
    sub_image = _nh.subscribe("/thermal_front/image_raw", 1, &ThermalPointCloudMapper::img_callback, this);
    
    _filteredPoints_visual = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/thermal/filtered", 1);
    _actual_visual = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/thermal/actual", 1);
}

bool ThermalPointCloudMapper::isBounded(int val, int lower, int upper) {
    return (lower <= val && val <= upper);
}

void ThermalPointCloudMapper::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::PCLPointCloud2 tmp_cloud;
    pcl_conversions::toPCL(*cloud_msg, tmp_cloud);
    pcl::fromPCLPointCloud2(tmp_cloud, *cloud_raw);
}

cv::Point2d ThermalPointCloudMapper::project3dToPixel(double x, double y, double z, int img_w, int img_h) {
    const double fx = img_w / (2 * tan(FOV / 2.0));
    const double fy = img_h / (2 * tan(FOV / 2.0));
    const double cx = (img_w / 2.0);
    const double cy = (img_h / 2.0);

    cv::Point2d uv_rect;
    uv_rect.x = (fx * x) / z + cx;
    uv_rect.y = (fy * y) / z + cy;
    return uv_rect;
}

void ThermalPointCloudMapper::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    if(_filteredPoints_visual.getNumSubscribers() == 0 && false) {
        return;
    }

    cv::Mat image_mono = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

    const int img_w = image_mono.cols;
    const int img_h = image_mono.rows;
    const int img_size = img_w * img_h;

    const uchar* data = image_mono.data;
    uchar max_ref = 0, min_ref = 255;
    for (int idx = 0; idx < img_size; idx++, data++) {
        if (max_ref < *data) {
            max_ref = *data;
        }
        if (min_ref > *data) {
            min_ref = *data;
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr forwarded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (pcl::PointXYZ &pt : cloud_raw->points) {

        forwarded_cloud->push_back(pt);

        cv::Point2d uv = project3dToPixel(pt.x, pt.y, pt.z, img_w, img_h);
        if (!isBounded(uv.x, 0, img_w)
            || !isBounded(uv.y, 0, img_h)) {
            continue;
        }

        const int red = ((image_mono.at<uchar>(uv) - min_ref) / (max_ref - min_ref + 0.01)) * 255;
        pcl::PointXYZRGB out_pt(red, 0, 255 - red);
        out_pt.x = pt.x;
        out_pt.y = pt.y;
        out_pt.z = pt.z;

        ROS_INFO("PT [%f, %f, %f] -> [%d, %d]", pt.x, pt.y, pt.z, (int)uv.x, (int)uv.y);
        processed_cloud->push_back(out_pt);
    }

    ROS_INFO("RANGE: [%d -> %d]", min_ref, max_ref);
    ROS_INFO("%d pt(s) matched", (int)processed_cloud->size());

    //For debugging
    processed_cloud->header.frame_id = "base_link";
    _filteredPoints_visual.publish(processed_cloud);

    forwarded_cloud->header.frame_id = "base_link";
    _actual_visual.publish(forwarded_cloud);
}
