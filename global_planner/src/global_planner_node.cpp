#include <amapper/grid.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>

AMapper::Grid grid;
tf::TransformListener* listener;


void planRoute(double start_x, double start_y, double goal_x, double goal_y) {
    //TODO Implement A* or Dijkstra
}

void onRecieveNewPoint(geometry_msgs::PointStamped goal) {
    
    //Acquire robot pose
    tf::StampedTransform robot_pose;
    try {
        listener->waitForTransform("X1/base_link", grid.getFrameId(), goal.header.stamp, ros::Duration(1.0));
        listener->lookupTransform("X1/base_link", grid.getFrameId(), goal.header.stamp, robot_pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("Failed to transform node %s", ex.what());
        return;
    }

    if(goal.header.frame_id != grid.getFrameId()) {
        ROS_ERROR("Please pass the goal in %s frame for now", grid.getFrameId());
    }

    auto start_x = grid.toXIndex(robot_pose.getOrigin().x());
    auto start_y = grid.toYIndex(robot_pose.getOrigin().x());

    auto goal_x = grid.toXIndex(goal.point.x);
    auto goal_y = grid.toYIndex(goal.point.y);
}

void onRecieveMap(nav_msgs::OccupancyGrid occupancy_map){
    grid = AMapper::Grid(occupancy_map);
}

void onReachDestination(std_msgs::Int8 status) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;
    listener = new tf::TransformListener;
    ros::Subscriber command_sub = nh.subscribe("/goal", 1, onRecieveNewPoint);
    ros::Subscriber map_sub = nh.subscribe("/global_map", 1, onRecieveMap);
    ros::Subscriber feedback_sub = nh.subscribe("/feedback", 1, onReachDestination);

    while(ros::ok()) {
        ros::spinOnce();
    }
} 