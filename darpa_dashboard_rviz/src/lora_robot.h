#ifndef LORA_ROBOT_H
#define LORA_ROBOT_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

namespace darpa_dashboard
{

class LoraRobot
{
public:
  LoraRobot(int);
  void drop();

private:
  int robot_id;

  ros::Publisher drop_lora_publisher;

  ros::NodeHandle nh;
};

}

#endif 
