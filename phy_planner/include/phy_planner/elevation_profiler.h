#ifndef __PHY_PLANNER_ELEV_PROF__
#define __PHY_PLANNER_ELEV_PROF__

#include <ros/ros.h>
#include <amapper/elevation_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <unordered_map>

#define WORLD_FRAME "world"
#define ROBOT_FRAME "X1/base_link"
#define ROBOT_HEIGHT 0.3

namespace phy_planner {
class ElevationProfiler {
public:
    ElevationProfiler(ElevationProfiler* profiler);
    ElevationProfiler(std::string topicname, std::string world_frame, std::string robot_frame, float robot_height);
    void onPCLReceived(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_msg);
    boost::shared_ptr<AMapper::ElevationGrid> final_map;
private:
    std::string world_frame;
    std::string robot_frame;
    float robot_height;
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    tf::TransformListener listener;
};


};
#endif