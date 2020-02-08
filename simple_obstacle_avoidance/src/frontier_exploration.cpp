#include <ros/ros.h>
#include <tf/tf.h>
#include <random>
#include <amapper/raytracers/NoClearingRaytracer.h>
#include <amapper/devices/laser_scanner.h>
#include <amapper/cluster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        std::default_random_engine gen;

enum State {
    SELECTING_TARGET, MOVE_TO_TARGET, SELECT_RECOVERY_TARGET, RECOVERY, E_STOPPED
};

State state = SELECTING_TARGET;

AMapper::Grid *grid, *frontier, *frontierScore;
AMapper::LaserScanner ls;
AMapper::NoClearingRaytracer rt;
MoveBaseClient* moveBaseClient;

ros::Time lastSent;

ros::Subscriber estopSubscriber;
ros::Subscriber startSubscriber;
ros::Subscriber imuSubscriber;
ros::Subscriber sonarSubscriber;

void onEstopRecieved(std_msgs::String estop){
    state = E_STOPPED;
    moveBaseClient->cancelGoal();
}

void onStartRecieved(std_msgs::String start) {
    state = SELECTING_TARGET;
}

void onSonarRecieved(sensor_msgs::Range range) {
    if(range.range > 0.4) {
        state = RECOVERY;
        moveBaseClient->cancelGoal();
        //Pick another goal.
    }
    //
    /*if(range.range < ){

    }*/
}

void onImuRecieved(sensor_msgs::Imu imu_reading) {
    Eigen::Vector3f acceleration(imu_reading.linear_acceleration.x, imu_reading.linear_acceleration.y, imu_reading.linear_acceleration.z);
    
}

uint8_t score(Eigen::Vector2i pt){
    float dx = pt.x() - frontierScore->gridWidth/2;
    float dy = pt.y() - frontierScore->gridHeight/2;
    float angle = 90*(abs(atan2(dy, dx)))/M_PI + 10;
    float preference = 1;
    if(atan2(dy, dx) <= -0.1){
        preference = 0.9;
    }
    return angle*preference;
}

void doneCb(const actionlib::SimpleClientGoalState& goalState, const move_base_msgs::MoveBaseResultConstPtr& result) {
    state = SELECTING_TARGET;
    ROS_INFO("%s", goalState.getText().c_str());
    if("Failed to find a valid plan. Even after executing recovery behaviors." == goalState.getText() 
    || "Failed to find a valid control. Even after executing recovery behaviors." == goalState.getText()) {
        state = SELECT_RECOVERY_TARGET;
    }
}
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& ptr){

}
void activeCb() {

}

ros::Publisher pub, frontiers, frontierScorePub;
void onLaserScan (sensor_msgs::LaserScan lscan) {

    //Sets the frameID for the maps
    grid->setFrameId(lscan.header.frame_id);
    frontier->setFrameId(lscan.header.frame_id);
    frontierScore->setFrameId(lscan.header.frame_id);

    //Clear the grid
    grid->clear();

    //Plot the laser scan
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    ls.plotgrid(transform, *grid, lscan);

    //Inflate it
    AMapper::Grid* inflated = AMapper::inflate(*grid, 3);
    pub.publish(inflated->toOccupancyGrid());

    //Get frontiers
    for(int x = 0; x < inflated->gridWidth ; x++) {
        for(int y = 0; y < inflated->gridHeight; y++) {
            frontier->data[y][x] = 0;
            if(inflated->data[y][x] == 0){
                for(int i = -1; i <2; i++){
                    for(int j = -1; j <2; j++){
                        int adjY = y + i;
                        int adjX = x + j;
                        if(!grid->isWithinGridCellMap(adjX, adjY))
                            continue;
                        if(inflated->data[adjY][adjX] == -1)
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
        //outlier rejection
        if(cluster.size() < 10) continue; 
        
        //Score each candidate point
        for(Eigen::Vector2i pt : cluster){

            if(pt.x() == frontierScore->gridWidth/2 && pt.x() == frontierScore->gridHeight/2){
                continue;
            }

            //For frontier purposes
            frontierScore->data[pt.y()][pt.x()] = score(pt);

            //Select best goal
            if(max_score < score(pt)) {
                max_score = score(pt);
                optimum_choice = pt;
                
            }
        }
    }
    //Build the goal message
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header = lscan.header;
    goal.target_pose.pose.position.x = frontierScore->fromXIndex(optimum_choice.x())*0.5;
    goal.target_pose.pose.position.y = frontierScore->fromYIndex(optimum_choice.y())*0.5;
    goal.target_pose.pose.position.z = 0;
    float targetYaw = atan2(frontierScore->fromYIndex(optimum_choice.y()), frontierScore->fromXIndex(optimum_choice.x()));
    tf::Quaternion targetYawQt(tf::Vector3(0,0,1), targetYaw);
    goal.target_pose.pose.orientation.x = targetYawQt.x();
    goal.target_pose.pose.orientation.y = targetYawQt.y();
    goal.target_pose.pose.orientation.z = targetYawQt.z();
    goal.target_pose.pose.orientation.w = targetYawQt.w();
    //Send the goal
    if(state == SELECTING_TARGET) {
        ROS_INFO("sending goal of %f,%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        moveBaseClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        state = MOVE_TO_TARGET;
        lastSent = ros::Time::now();
    }
    if(state == SELECT_RECOVERY_TARGET) {
        ROS_INFO("Recovery attempt");

        std::vector<AMapper::Cluster> validClusters;
        for(AMapper::Cluster cluster : clusters) {
            //outlier rejection
            if(cluster.size() < 20) continue; 
            validClusters.push_back(cluster);
        }
        std::uniform_int_distribution<int> distr(0,validClusters.size()-1);
        AMapper::Cluster cluster = validClusters[distr(gen)];
        std::uniform_int_distribution<int> distr2(0, cluster.size()-1);

        optimum_choice = cluster[distr2(gen)];
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header = lscan.header;
        goal.target_pose.pose.position.x = frontierScore->fromXIndex(optimum_choice.x())*0.7;
        goal.target_pose.pose.position.y = frontierScore->fromYIndex(optimum_choice.y())*0.7;
        goal.target_pose.pose.position.z = 0;
        float targetYaw = atan2(frontierScore->fromYIndex(optimum_choice.y()), frontierScore->fromXIndex(optimum_choice.x()));
        tf::Quaternion targetYawQt(tf::Vector3(0,0,1), targetYaw);
        goal.target_pose.pose.orientation.x = targetYawQt.x();
        goal.target_pose.pose.orientation.y = targetYawQt.y();
        goal.target_pose.pose.orientation.z = targetYawQt.z();
        goal.target_pose.pose.orientation.w = targetYawQt.w();
        moveBaseClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        state = MOVE_TO_TARGET;
        lastSent = ros::Time::now();
    }
    if(state == MOVE_TO_TARGET) {
        if(ros::Time::now()-lastSent > ros::Duration(30)) {
            state = SELECTING_TARGET;
        }
    }
    delete inflated;
}

int main(int argc, char** argv) {
    
    //Allocate nessecary AMapper grids
    grid = new AMapper::Grid(0,0,150,150,0.1);
    frontier = new AMapper::Grid(0,0,150,150,0.1);
    frontierScore = new AMapper::Grid(0, 0, 150, 150, 0.1);
   
    //Set The raytracer
    ls.setRaytracer(rt);
    
    //ROS subscribers
    ros::init(argc, argv, "LaserScanExampleNode");
    ros::NodeHandle nh;

    //Debugging publishers
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/out", 10);
    frontiers = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/frontiers", 10);
    frontierScorePub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/frontiers/score", 10);
    
    //Subscribers
    ros::Subscriber sub = nh.subscribe("/scan", 1, onLaserScan);
    ros::Subscriber estopSub= nh.subscribe<std_msgs::String>("e_stop", 10, onEstopRecieved);
    ros::Subscriber start = nh.subscribe<std_msgs::String>("start", 10, onStartRecieved);

    //Movebase initiallization
    moveBaseClient = new MoveBaseClient("move_base", true);
    while(!moveBaseClient->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while(ros::ok()) {
        ros::spinOnce();
        frontiers.publish(frontier->toOccupancyGrid());
        frontierScorePub.publish(frontierScore->toOccupancyGrid());
    }
    
    //Clean up
    delete grid;
    delete frontier;
    delete frontierScore;

    return 0;
}
