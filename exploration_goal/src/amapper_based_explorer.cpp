#include <ros/ros.h>
#include <map_merge/laser_operations.h>
#include <amapper/grid.h>
#include <amapper/raytracers/NoClearingRaytracer.h> //TODO Implement custom raytracer
#include <tf/transform_listener.h>
#include <tf2/exceptions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

AMapper::Grid* grid;
tf::TransformListener* listener;
ros::Publisher debug_occupancy_topic;
//Private API inside laser_operations
pcl::PointXYZ scanPointToPointCloud(sensor_msgs::LaserScan& scan, int index, double azimuth);

void onRecievePointCloud(pcl::PointCloud<pcl::PointXYZ> pcloud){
    
    //Find current robot pose
    tf::StampedTransform robot_pose;
    try {
        ros::Time time = pcl_conversions::fromPCL(pcloud.header.stamp);
        listener->waitForTransform(grid->getFrameId(), "X1/base_link", time, ros::Duration(1.0));
        listener->lookupTransform(grid->getFrameId(), "X1/base_link", time, robot_pose);
    }catch(tf::LookupException ex){
        ROS_ERROR("Failed to lookup %s", ex.what());
        return;
    }catch(tf2::TransformException ex ){
        ROS_ERROR("Failed to lookup %s", ex.what());
        return;
    } 

    //Look for steep points on LiDAR
    static LidarScan scan; //Static is used to preserve the lidar settings so next time we don't need to realliocate memory
    decomposeLidarScanIntoPlanes(pcloud, scan);
    std::reverse(scan.begin(), scan.end());    
    float start_ang = scan[0].scan.angle_min, increment = scan[0].scan.angle_increment; 
    std::vector<float> steep_paths(scan[0].scan.ranges.size(), 0);
    
    for(int i = 0; i < scan.size() -1; i++){
        for(int j = 0; j < scan[i].scan.ranges.size(); j++){
            if(!std::isfinite(scan[i].scan.ranges[j]) || !std::isfinite(scan[i+1].scan.ranges[j])) continue;
            
            auto p1 = scanPointToPointCloud(scan[i].scan, j, scan[i].azimuth);
            auto p2 = scanPointToPointCloud(scan[i+1].scan, j, scan[i+1].azimuth);
            
            if(p1.z >  2 && p2.z >2) continue;

            tf::Vector3 v1(p1.x, p1.y, p1.z);
            tf::Vector3 v2(p2.x, p2.y, p2.z);
            auto world1 = robot_pose*v1;
            auto world2 = robot_pose*v2;

            auto gradient = world2 - world1;
            auto norm = sqrt(gradient.x()*gradient.x() + gradient.y()*gradient.y());
            auto angle = abs(atan2(gradient.z(), norm));

            if(angle > 0.61) {
                steep_paths[j] = std::min(scan[i].scan.ranges[j], scan[i+1].scan.ranges[j]); 
            }
        }
    }

    //Raytracer 
    AMapper::NoClearingRaytracer raytracer;
    float curr_angle = start_ang;
    auto origin = robot_pose.getOrigin();
    auto rotation = robot_pose.getRotation();
    auto robot_rotation= tf::getYaw(rotation);
    Eigen::Vector2f center(origin.x(), origin.y());
    for(int i = 0; i < steep_paths.size(); i++) {

        Eigen::Vector2f end(origin.x() + steep_paths.at(i)*cos(robot_rotation+curr_angle), origin.y() + steep_paths.at(i)*sin(robot_rotation+curr_angle));
        auto y = grid->toYIndex(end.y());
        auto x = grid->toXIndex(end.x());
       
        curr_angle += increment;
        raytracer.rayTrace(*grid, center, end);
        if(!grid->isWithinGridCellMap(x, y)) continue;
        if(steep_paths[i] != 0)grid->data[y][x] = 255;
    }

    debug_occupancy_topic.publish(grid->toOccupancyGrid());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "amapperexplorer");
    ros::NodeHandle nh;
    debug_occupancy_topic = nh.advertise<nav_msgs::OccupancyGrid>("/X1/steepness_grid", 1);
    ros::Subscriber sub = nh.subscribe("/X1/points", 1, onRecievePointCloud);
    listener = new tf::TransformListener();
    grid = new AMapper::Grid(0,0,5000,5000,1);
    grid->setFrameId("X1/world");
    ros::spin();
}