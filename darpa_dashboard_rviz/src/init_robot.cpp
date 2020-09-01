#include "init_robot.h"
#include "std_msgs/String.h"

namespace darpa_dashboard
{
InitRobot::InitRobot(int i):
  robot_id(i)
{
  std::stringstream stop_topic;
  stop_topic << "/X" << robot_id << "/e_stop";
  std::stringstream start_topic;
  start_topic << "/X" << robot_id << "/start";
  start_robot_publisher = nh.advertise<std_msgs::String>(start_topic.str(), 1 );
  stop_robot_publisher = nh.advertise<std_msgs::String>(stop_topic.str(), 1 );
}

void InitRobot::stop() 
{
  std_msgs::String msg;
  msg.data = std::to_string(robot_id);
  stop_robot_publisher.publish(msg);
}

void InitRobot::start() 
{
  std_msgs::String msg;
  msg.data = std::to_string(robot_id);
  start_robot_publisher.publish(msg);
}

}