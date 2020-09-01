#ifndef TWIST_ROBOT_H
#define TWIST_ROBOT_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

namespace darpa_dashboard
{

class TwistRobot
{
public:
  TwistRobot(int);
  void move(double, double, double);

private:
  int robot_id;

  ros::Publisher move_publisher;

  ros::NodeHandle nh;
};

}

#endif 
