#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <mission_planner/RosThread.h>

/***
 * A robot represents 1 individual vehicle inm the fleet
 * For now this class is tailored only to UGVs
 * 
 */ 
class Robot {

public:
    Robot(ros::NodeHandle nh, uint8_t robotNum);
    ~Robot();
    ROSThread rosthread;
};
#endif