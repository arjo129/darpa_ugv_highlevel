#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <mission_planner/RosThread.h>
#include <QGraphicsItem>

/***
 * A robot represents 1 individual vehicle inm the fleet
 * For now this class is tailored only to UGVs
 * 
 */ 
class Robot {

public:
    Robot(ros::NodeHandle nh, int robotNum);
    ~Robot();
    ROSThread* rosthread;
    std::vector<QGraphicsPixmapItem*> laserscans;
};
#endif