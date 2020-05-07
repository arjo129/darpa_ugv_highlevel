#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <amapper/elevation_grid.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define WORLD_FRAME "world"
#define ROBOT_FRAME "X1/base_link"
#define ROBOT_HEIGHT 0.3


std::vector<AMapper::Coordinate> getNeighbours(AMapper::Coordinate coordinate, int radius = 3) {
    std::vector<AMapper::Coordinate> neighbours;
    for(int i = -radius; i < radius; i++) {
        for(int j = -radius; j < radius; j++) {
            AMapper::Coordinate neighbour;
            neighbour.first = coordinate.first + i;
            neighbour.second = coordinate.second + j;
            neighbours.push_back(neighbour);
        }    
    }
    return neighbours;
}

/**
 * Looks for traversable ground points within a cell. A ground point is defined as follows:
 *  - It is the topmost point before a gap that is more than the height of the robot
 *  OR
 *  - It is the last point of the set
 */
std::vector<double> findTraversablePoints(double robot_height, std::set<double> point_sorter) {
    double last_point = -INFINITY;
    bool first_round = true;
    std::vector<double> ground_points;
    for (double pt: point_sorter) {
        if (first_round) {
            last_point = pt;
            first_round = false;
            continue;
        }
        auto diff = pt - last_point;
        if(diff >robot_height) {
            ground_points.push_back(last_point);
        }
        last_point = pt;
    }
    ground_points.push_back(last_point);
    return ground_points;
}

/**
 * find the lowest ground point
 * TODO: This is not the ideal method of estimating ground points
 */ 
double getLowestGroundPoint(std::vector<double> point){
    if (point.size() > 0)
        return point[0];
    return 0;
}


class NormalEstimator {
public:
    NormalEstimator();
private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher grid_pub;
    tf::TransformListener listener;
    boost::shared_ptr<AMapper::ElevationGrid> final_map;
    void onPCLReceived(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_msg);
};

NormalEstimator::NormalEstimator(): listener(nh_, ros::Duration(30)) {
    pcl_sub_ = nh_.subscribe("/X1/rgbd_camera/depth/points", 1, &NormalEstimator::onPCLReceived, this);
    grid_pub = nh_.advertise<amapper::ElevationGridMsg>("/plane_segmentation/grid_map", 1);
    final_map = boost::make_shared<AMapper::ElevationGrid>();
    final_map->setFrameId(WORLD_FRAME);
}

void NormalEstimator::onPCLReceived(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  pcl_msg) {

    pcl::PointCloud<pcl::PointXYZ> transformed_point_cloud;
    bool res = pcl_ros::transformPointCloud(WORLD_FRAME, *pcl_msg, transformed_point_cloud, listener);
    if(!res)
        return;


    AMapper::GenericCoordinateStorage<std::set<double>> point_sorter;
    for(auto pt: transformed_point_cloud){
        AMapper::Coordinate coord(final_map->toXIndex(pt.x), final_map->toYIndex(pt.y));
        point_sorter[coord].insert(pt.z);        
    }

    for(auto point_candidates: point_sorter){
        auto candidate_points = findTraversablePoints(ROBOT_HEIGHT, point_candidates.second);
        if(candidate_points.size() == 0) continue;
        auto pt = getLowestGroundPoint(candidate_points);
        amapper::ElevationPoint elevPt;
        elevPt.x = final_map->fromXIndex(point_candidates.first.first);
        elevPt.y = final_map->fromYIndex(point_candidates.first.second); 
        elevPt.elevation = pt;
        final_map->add(elevPt);
    }

    grid_pub.publish(final_map->toRosMsg());
}


int main(int argc,char** argv) {
    ros::init(argc, argv, "realsense_normal_estimates");
    NormalEstimator ne;
    ros::spin();
}