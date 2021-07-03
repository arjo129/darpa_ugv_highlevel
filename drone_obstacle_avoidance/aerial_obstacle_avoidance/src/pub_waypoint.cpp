#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>

#include <string>
#include <math.h>
#include <vector>

using namespace std;

class WaypointPub{

private:
    ros::Publisher goal_pub;
    ros::Subscriber waypoints_sub, goal_status_sub;
    ros::Subscriber odom_sub;
    tf::TransformListener tf_listener_;
    std::string base_link_, world_link_, waypoints_topic_, goal_pub_topic_, goal_status_topic_;

    int rate, num_path_nodes, counter;
    nav_msgs::Path path;
    geometry_msgs::PointStamped goal_msg_gl; // goal in global frame
    bool path_received;

public:
    WaypointPub(ros::NodeHandle &node) {
        ros::param::get("~base_link_", base_link_);
        ros::param::get("~world_link_", world_link_);
        ros::param::get("~rate", rate);
        ros::param::get("~goal_pub_topic_", goal_pub_topic_);
        
        goal_pub = node.advertise<geometry_msgs::PointStamped>(goal_pub_topic_, rate);
        waypoints_sub = node.subscribe("/waypoints", rate, &WaypointPub::WaypointsCallback, this);
        goal_status_sub = node.subscribe("/status", rate, &WaypointPub::GoalStatusCallback, this);
        path_received = false;
    }

    void WaypointsCallback(const nav_msgs::Path &path_msg){
        path = path_msg;
        path.header.stamp = ros::Time::now();;
        num_path_nodes = path_msg.poses.size();
        path_received = true;
        cout << "Waypoints received from A* planner." << "\n";
        counter = 1;
    } 

    void GoalStatusCallback(const std_msgs::Int8 &msg){
        
        if (path_received){
            int path_size = path.poses.size();

            if(msg.data != 0){ //goal is reached, hence pub next goal
    
                goal_msg_gl.header.frame_id = world_link_;
                goal_msg_gl.header.stamp = ros::Time::now();
                goal_msg_gl.point = path.poses[counter].pose.position;

                cout << "Publishing " << counter << "/" << path_size << " goal" << endl;
                goal_pub.publish(goal_msg_gl);
                counter ++;
            }
 
        }
    }   

};

int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint pub");
    
    ros::NodeHandle node;
    WaypointPub waypointPub(node);
    ros::spin();
    
    return 0;
}