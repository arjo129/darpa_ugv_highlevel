#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
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

using namespace std;

const double PI = 3.1415926;

string odometry_topic, waypoint_topic, map_frame;
double waypointXYRadius = 2.0;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 1.0;
bool sendSpeed = true;
bool sendBoundary = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;
float goalX = 0, goalY = 0, goalZ = 0;
int wayPointID;

void create_path(){
    pcl::PointXYZ point;
    point.x = 10;
    point.y = -4;
    point.z = 0;
    waypoints->push_back(point);
    point.x = 15;
    point.y = -4;
    point.z = 0;    
    waypoints->push_back(point);
    point.x = 15;
    point.y = -10;
    point.z = 0;
    waypoints->push_back(point);
    point.x = 15;
    point.y = -20;
    point.z = 0;
    waypoints->push_back(point);
    point.x = 15;
    point.y = -30;
    point.z = 0;
    waypoints->push_back(point);
    point.x = 18;
    point.y = -40;
    point.z = 0;
    waypoints->push_back(point);
    point.x = 18;
    point.y = -50;
    point.z = 0;
    waypoints->push_back(point);
    point.x = 21;
    point.y = -52;
    point.z = 0;
    waypoints->push_back(point);
    cout << "number of waypoints: " << waypoints->points.size() << endl;
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
    map_frame = pose->header.frame_id;
    curTime = pose->header.stamp.toSec();
    vehicleX = pose->pose.pose.position.x;
    vehicleY = pose->pose.pose.position.y;
    vehicleZ = pose->pose.pose.position.z;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypointPublisher");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    
    nhPrivate.getParam("odometry_topic", odometry_topic);
    nhPrivate.getParam("waypoint_topic", waypoint_topic);
    nhPrivate.getParam("frameRate", frameRate);

    ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> (odometry_topic, 10, poseHandler);
    ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped>(waypoint_topic, 1);


    create_path();
    int waypointSize = waypoints->points.size();

    geometry_msgs::PointStamped waypointMsgs;
    waypointMsgs.header.frame_id = map_frame;

    ros::Rate rate(100);
    while (ros::ok()){
        ros::spinOnce();
        
        float disX = vehicleX - waypoints->points[wayPointID].x;
        float disY = vehicleY - waypoints->points[wayPointID].y;
        // float disZ = vehicleZ - waypoints->points[wayPointID].z;
        float dist_to_waypoint = sqrt(disX * disX + disY * disY);

        // wait if current waypoint is reached
        if (dist_to_waypoint < waypointXYRadius && !isWaiting){
            waitTimeStart = curTime;
            isWaiting = true;
        }
        
        // move to the next waypoint after waiting is over
        if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
            wayPointID++;
            isWaiting = false;
        }        

        // publish waypoint msgs at certain frame rate
        if (curTime - waypointTime > 1.0 / frameRate) {
            if (!isWaiting) {
                waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
                waypointMsgs.point.x = waypoints->points[wayPointID].x;
                waypointMsgs.point.y = waypoints->points[wayPointID].y;
                waypointMsgs.point.z = waypoints->points[wayPointID].z;
                pubWaypoint.publish(waypointMsgs);
                cout << "Publishing " << wayPointID << "/" << waypointSize << " waypointID" << endl;
            }

            waypointTime = curTime;
        }

        rate.sleep();
    }


    return 0;
}