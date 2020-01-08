#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

/**
 * Creates a localmap with Laser scan and position information.
 */ 
struct RobotPoseLocalMap {
    sensor_msgs::LaserScan scan;
    nav_msgs::Odometry odom;
};
/***
 * A robot represents 1 individual vehicle inm the fleet
 * For now this class is tailored only to UGVs
 * 
 */ 
class Robot {
    std::vector<RobotPoseLocalMap> robotMaps;

public:
    Robot(std::string _namespace);
    
};
#endif