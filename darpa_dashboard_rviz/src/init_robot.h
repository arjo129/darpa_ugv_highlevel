#ifndef INIT_ROBOT_H
#define INIT_ROBOT_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

namespace darpa_dashboard
{

class InitRobot
{
public:
  InitRobot(int);
  void start();
  void stop();

private:
  int robot_id;

  ros::Publisher start_robot_publisher;
  ros::Publisher stop_robot_publisher;

  ros::NodeHandle nh;
};

}

#endif 
