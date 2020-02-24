#include <mission_planner/Robot.h>

Robot::Robot(ros::NodeHandle nh, int robotNum) : rosthread(nh, robotNum)
{
    rosthread.start();                                        
}

Robot::~Robot() {}