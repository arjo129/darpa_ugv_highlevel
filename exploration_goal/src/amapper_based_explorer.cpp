#define ANGLE 0.6 //Radians
#define DECAY_RATE 5 // Between 0 -100. 100 should be amultiple of the number
#define DISTANCE_TO_CLEAR 10//
#include <ros/ros.h>
#include <map_merge/laser_operations.h>
#include <amapper/grid.h>
#include <amapper/raytracers/NaiveRaytracer.h> //TODO Implement custom raytracer
#include <amapper/RayTracer.h>
#include <tf/transform_listener.h>
#include <tf2/exceptions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_set>
AMapper::Grid* grid, last_sent_grid;
tf::TransformListener* listener;
ros::Publisher debug_occupancy_topic;
//Private API inside laser_operations
pcl::PointXYZ scanPointToPointCloud(sensor_msgs::LaserScan& scan, int index, double azimuth);
struct pair_hash
{
	template <class T1, class T2>
	std::size_t operator () (std::pair<T1, T2> const &pair) const
	{
		std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);

		return h1 ^ h2;
	}
};

class MultiLayerRaytracer : public AMapper::RayTracer {
    public:
    void reset(){
        current_scan.clear();
    }
    private:
    std::unordered_set<std::pair<int,int>, pair_hash> current_scan;
    /**
     * @param occupancy grid
     * @param centroid Current robot position
     * @param X, Y - Target coordinates
     */ 
    void plotFreeSpace(AMapper::Grid& grid, Eigen::Vector2i centroid, int x, int y) override {
        
        auto exists = current_scan.find(std::make_pair(x,y));
        
        Eigen::Vector2i v(x,y);
        
        if(exists == current_scan.end()){
            if(grid.data[y][x] <= 0 && (centroid -v).norm()*grid.getResolution() < DISTANCE_TO_CLEAR)
                grid.data[y][x] = 0;
            else if(grid.data[y][x] > 0 && (centroid -v).norm()*grid.getResolution() > 3 && (centroid -v).norm()*grid.getResolution() < DISTANCE_TO_CLEAR)
                grid.data[y][x] -= DECAY_RATE;
        }

        current_scan.insert(std::make_pair(x,y));
    }    
};
MultiLayerRaytracer raytracer;
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
    LidarScan full_scan, scan; //Static is used to preserve the lidar settings so next time we don't need to reallocate memory
    decomposeLidarScanIntoPlanes(pcloud, full_scan);
    if(full_scan.size() < 1) return;
    if(full_scan[0].scan.ranges.size() > 2000){ 
        downsample(full_scan, scan, 4);
    }
    else{
        scan = full_scan;
    }

    std::reverse(scan.begin(), scan.end());    
    float start_ang = scan[0].scan.angle_min, increment = scan[0].scan.angle_increment; 
    std::vector<float> steep_paths(scan[0].scan.ranges.size(), 0);
    for(int i = 0; i < scan.size() -1; i++){
        for(int j = 0; j < scan[i].scan.ranges.size(); j++){
            if(!std::isfinite(scan[i].scan.ranges[j]) || !std::isfinite(scan[i+1].scan.ranges[j]) || 
            scan[i].scan.ranges[j] > 120 || scan[i+1].scan.ranges[j] > 120 
            || scan[i].scan.ranges[j] < 0 || scan[i+1].scan.ranges[j] < 0 ) continue;
            
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

            auto p1_dist_sq = p1.x*p1.x + p1.y*p1.y;
            auto p2_dist_sq = p2.x*p2.x + p2.y*p2.y; 

            if(angle > ANGLE) {
                if (steep_paths[j] != 0){
                    steep_paths[j] = std::min(std::min(scan[i].scan.ranges[j], scan[i+1].scan.ranges[j]), steep_paths[j]);
                }
                else {
                    steep_paths[j] = std::min(scan[i].scan.ranges[j], scan[i+1].scan.ranges[j]);
                }
            }

        }
    }

    //Raytracer 
    
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
        if(steep_paths[i] != 0 && (center - end).norm() < DISTANCE_TO_CLEAR*1.3)grid->data[y][x] = 255;
    }

    for(float i = -50; i < 50; i+= grid->getResolution()){
        auto y = grid->toYIndex(i);
        auto x = grid->toXIndex(-0.3);
        grid->data[y][x] = 100; // Don't go exploring the staging area you peice of shit
    }
    raytracer.reset();

}

void onRecieveUpdate(nav_msgs::OccupancyGrid grid) {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amapperexplorer");
    ros::NodeHandle nh;
    debug_occupancy_topic = nh.advertise<nav_msgs::OccupancyGrid>("steepness_grid", 1);
    ros::Subscriber sub = nh.subscribe("/X1/points", 1, onRecievePointCloud);
    listener = new tf::TransformListener();
    grid = new AMapper::Grid(0,0,5000,5000,0.3);
    grid->setFrameId("X1/world");
    ros::Duration start_delay(10);
    start_delay.sleep();
    ros::Rate r(2);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
        debug_occupancy_topic.publish(grid->toOccupancyGrid());
    }
}