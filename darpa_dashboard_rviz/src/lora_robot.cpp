#include "lora_robot.h"
#include "std_msgs/String.h"

namespace darpa_dashboard
{
LoraRobot::LoraRobot(int i):
  robot_id(i)
{
  std::stringstream drop_artifact_topic;
  drop_artifact_topic << "/X" << robot_id << "/dropper";
  drop_lora_publisher = nh.advertise<std_msgs::String>(drop_artifact_topic.str(), 1 );
}

void LoraRobot::drop() 
{
  std_msgs::String msg;
  msg.data = "lora";
  drop_lora_publisher.publish(msg);
}

}