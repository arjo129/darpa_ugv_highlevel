#include <ros/ros.h>
#include <tf/tf.h>
#include <amapper/raytracers/NaiveRaytracer.h>
#include <amapper/devices/laser_scanner.h>

AMapper::Grid *grid, *frontier;
AMapper::LaserScanner ls;
AMapper::NaiveRaytracer rt;
ros::Publisher pub, frontiers;

void onLaserScan (sensor_msgs::LaserScan lscan) {
    grid->clear();
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    ls.plotgrid(transform, *grid, lscan);
    pub.publish(grid->toOccupancyGrid());
    for(int x = 0; x < grid->gridWidth ; x++) {
        for(int y = 0; y < grid->gridHeight; y++) {
            frontier->data[y][x] = 0;
            if(grid->data[y][x] == 0){
                for(int i = -1; i <2; i++){
                    for(int j = -1; j <2; j++){
                        int adjY = y + i;
                        int adjX = x + j;
                        if(!grid->isWithinGridCellMap(adjX, adjY))
                            continue;
                        if(grid->data[adjY][adjX] == -1)
                            frontier->data[y][x] = 100;
                    }
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    grid = new AMapper::Grid(0,0,200,200,0.1);
    frontier = new AMapper::Grid(0,0,200,200,0.1);
    grid->setFrameId(std::string("laser"));
    frontier->setFrameId(std::string("laser"));
    ls.setRaytracer(rt);
    ros::init(argc, argv, "LaserScanExampleNode");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/out", 10);
    frontiers = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/frontiers", 10);
    ros::Subscriber sub = nh.subscribe("/scan", 1, onLaserScan);
    while(ros::ok()) {
        ros::spinOnce();
        frontiers.publish(frontier->toOccupancyGrid());
    }
    delete grid;
    delete frontier;
    return 0;
}