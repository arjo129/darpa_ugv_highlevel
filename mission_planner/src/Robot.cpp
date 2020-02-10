#include <mission_planner/Robot.h>

Robot::Robot(ros::NodeHandle nh, uint8_t robotNum) : rosthread(nh, robotNum) 
{
    rosthread.start();                                        
}

Robot::~Robot() {}