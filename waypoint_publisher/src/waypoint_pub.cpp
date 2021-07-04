#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <string>

using namespace std;

bool waypoints_initialized = false;

class waypointPub{

private:
    ros::NodeHandle nh;
    ros::Publisher pubWaypoint, pubGoalReachStatus;
    ros::Subscriber subPose, path_sub;

    string odometry_topic, map_frame, global_path_topic, waypoint_topic, goal_task_status;
    double waypointXYRadius, waitTime, waitTimeStart, frameRate, speed, curTime, waypointTime, PI;
    bool isWaiting, sendSpeed, sendBoundary, newGoal;
    float vehicleX, vehicleY, vehicleZ, goalX, goalY, goalZ;
    int wayPointID;
    pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints;
    geometry_msgs::PointStamped waypointMsg;

public:
    waypointPub():
        nh("~")
    {
        ros::param::get("~odometry_topic", odometry_topic);
        ros::param::get("~global_path_topic", global_path_topic);
        ros::param::get("~waypoint_topic", waypoint_topic);
        ros::param::get("~goal_task_status", goal_task_status);
        ros::param::get("~waypointXYRadius", waypointXYRadius);
        ros::param::get("~frameRate", frameRate);
        ros::param::get("~map_frame", map_frame);

        subPose = nh.subscribe (odometry_topic, 10, &waypointPub::poseHandler, this);
        path_sub = nh.subscribe (global_path_topic, 10, &waypointPub::pathCallback, this);
        pubWaypoint = nh.advertise<geometry_msgs::PointStamped>(waypoint_topic, 1);
        pubGoalReachStatus = nh.advertise<std_msgs::Empty>(goal_task_status, 1);

        waitTime = 0;
        waitTimeStart = 0;
        isWaiting = false;
        newGoal = true;
        frameRate = 5.0;
        speed = 1.0;
        sendSpeed = true;
        sendBoundary = true;
        vehicleX = 0, vehicleY = 0, vehicleZ = 0;
        curTime = 0, waypointTime = 0;
        goalX = 0, goalY = 0, goalZ = 0;
        wayPointID = 0;
        PI = 3.1415926;

        waypoints.reset(new pcl::PointCloud<pcl::PointXYZ>()); 

        // create_path(); // fixed path, only designed for qualification circuit. 
    }

    void create_path(){
        waypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ point;
        waypoints_initialized = true;

        point.x = 0;
        point.y = -2;
        point.z = 3;
        waypoints->push_back(point);
        point.x = 0;
        point.y = -4;
        point.z = 3;   
        waypoints->push_back(point);
        point.x = 0;
        point.y = -6;
        point.z = 3;
        waypoints->push_back(point);
        point.x = 2;
        point.y = -6;
        point.z = 5;
        waypoints->push_back(point);
        point.x = 0;
        point.y = -2;
        point.z = 3;
        waypoints->push_back(point);
        cout << "number of waypoints: " << waypoints->points.size() << endl;
    }

    // vehicle pose callback function
    void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
    {
        // cout << "Odom receieved" << endl;
        curTime = pose->header.stamp.toSec();
        // for LOAM since it is using diff frame convention

        vehicleX = pose->pose.pose.position.z;
        vehicleY = pose->pose.pose.position.x;
        vehicleZ = pose->pose.pose.position.y;    

        // RH convention
        // vehicleX = pose->pose.pose.position.x;
        // vehicleY = pose->pose.pose.position.y;
        // vehicleZ = pose->pose.pose.position.z;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg){
        waypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
        waypoints -> clear();
        pcl::PointXYZ point;
        waypoints_initialized = true;
        
        map_frame = msg->header.frame_id;

        for (int i=0; i< msg->poses.size(); ++i){
            point.x = msg->poses[i].pose.position.x;
            point.y = msg->poses[i].pose.position.y;
            point.z = msg->poses[i].pose.position.z;
            waypoints->push_back(point);
        }
        cout << msg->poses.size() << " waypoints received from A* planner." << endl;

    } 

    void move(){
        int waypointSize = waypoints->points.size();

        waypointMsg.header.frame_id = map_frame;
        float disX = vehicleX - waypoints->points[wayPointID].x;
        float disY = vehicleY - waypoints->points[wayPointID].y;
        float disZ = vehicleZ - waypoints->points[wayPointID].z;
        float dist_to_waypoint = sqrt(disX*disX + disY*disY + disZ*disZ);
        cout << "Distance to goal: " << dist_to_waypoint << endl;     
    
        // wait if current waypoint is reached
        if (dist_to_waypoint < waypointXYRadius && !isWaiting){
            waitTimeStart = curTime;
            isWaiting = true;

            if (wayPointID == waypointSize - 1){
                std_msgs::Empty empty;
                pubGoalReachStatus.publish(empty);
                cout << "Task of moving to goal completed!" << endl;
            }
        }
        
        // move to the next waypoint after waiting is over
        if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
            wayPointID++;
            isWaiting = false;
            newGoal = true;
        }   

        // publish waypoint msgs at certain frame rate
        if (curTime - waypointTime > 1.0 / frameRate && newGoal) {
            if (!isWaiting) {
                newGoal = false;
                waypointMsg.header.stamp = ros::Time().fromSec(curTime);
                waypointMsg.point.x = waypoints->points[wayPointID].x;
                waypointMsg.point.y = waypoints->points[wayPointID].y;
                waypointMsg.point.z = waypoints->points[wayPointID].z;
                pubWaypoint.publish(waypointMsg);
                cout << "Publishing " << wayPointID + 1 << "/" << waypointSize << " waypointID" << endl;
            }
            
            waypointTime = curTime;
        }

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv,"waypointPublisher");

    ros::NodeHandle nh;
    waypointPub waypointPub;
    
    ros::Rate r(10);
    while (ros::ok()){
        ros::spinOnce();
        if (waypoints_initialized){
            waypointPub.move();
        }

        r.sleep();
    }

    return 0;
}