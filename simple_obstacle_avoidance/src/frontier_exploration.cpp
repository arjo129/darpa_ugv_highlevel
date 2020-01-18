#include <ros/ros.h>
#include <tf/tf.h>
#include <amapper/raytracers/NaiveRaytracer.h>
#include <amapper/devices/laser_scanner.h>
#include <amapper/cluster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum State {
    SELECTING_TARGET, MOVE_TO_TARGET
};

State state = SELECTING_TARGET;
AMapper::Grid *grid, *frontier, *frontierScore;
AMapper::LaserScanner ls;
AMapper::NaiveRaytracer rt;
MoveBaseClient* client;

uint8_t score(Eigen::Vector2i pt){
     float dx = pt.x() - frontierScore->gridWidth/2;
     float dy = pt.y() - frontierScore->gridHeight/2;
     float angle = 90*(M_PI - abs(atan2(dy, dx)))/M_PI + 10;
     return angle;
}

ros::Publisher pub, frontiers, frontierScorePub;
void onLaserScan (sensor_msgs::LaserScan lscan) {
    grid->clear();
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    ls.plotgrid(transform, *grid, lscan);
    pub.publish(grid->toOccupancyGrid());

    //Get frontiers
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

    //Get clusters
    std::vector<AMapper::Cluster> clusters = AMapper::getClusters(*frontier);
    for(int i = 0; i < frontierScore->gridWidth; i++){
        for(int j = 0; j < frontierScore->gridHeight; j++) {
            frontierScore->data[i][j] = 0;
        }
    }

    //get the best scoring candidate
    uint8_t max_score = 0;
    Eigen::Vector2i optimum_choice;
    for(AMapper::Cluster cluster : clusters) {
        if(cluster.size() < 10) continue;
        for(Eigen::Vector2i pt : cluster){
            frontierScore->data[pt.y()][pt.x()] = score(pt);
            if(max_score > score(pt)) {
                max_score = score(pt);
                optimum_choice = pt;
            }
        }
    }
    std::cout << "Selecting: " << optimum_choice << std::endl;
}

int main(int argc, char** argv) {
    grid = new AMapper::Grid(0,0,150,150,0.1);
    frontier = new AMapper::Grid(0,0,150,150,0.1);
    frontierScore = new AMapper::Grid(0, 0, 150, 150, 0.1);
    grid->setFrameId(std::string("/X1/base_link/frontlaser"));
    frontier->setFrameId(std::string("/X1/base_link/frontlaser"));
    ls.setRaytracer(rt);
    ros::init(argc, argv, "LaserScanExampleNode");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/out", 10);
    frontiers = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/frontiers", 10);
    frontierScorePub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/frontiers/score", 10);
    ros::Subscriber sub = nh.subscribe("/scan", 1, onLaserScan);
    while(ros::ok()) {
        ros::spinOnce();
        frontiers.publish(frontier->toOccupancyGrid());
        frontierScorePub.publish(frontierScore->toOccupancyGrid());
    }
    delete grid;
    delete frontier;
    return 0;
}