#include <ros/ros.h>
#include <tf/tf.h>
#include <amapper/raytracers/NaiveRaytracer.h>
#include <amapper/devices/laser_scanner.h>

AMapper::Grid* grid;
AMapper::LaserScanner ls;
AMapper::NaiveRaytracer rt;
ros::Publisher pub;

void onLaserScan (sensor_msgs::LaserScan lscan) {
    grid->clear();
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    ls.plotgrid(transform, *grid, lscan);
    pub.publish(grid->toOccupancyGrid());
}

int main(int argc, char** argv) {
    grid = new AMapper::Grid(0,0,400,400,0.1);
    grid->setFrameId(std::string("husky1/base_laser"));
    ls.setRaytracer(rt);
    ros::init(argc, argv, "LaserScanExampleNode");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/out", 10);
    ros::Subscriber sub = nh.subscribe("husky1/scan", 1, onLaserScan);
    while(ros::ok()) {
        ros::spinOnce();
    }
}