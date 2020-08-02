#include "twist_robot.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace darpa_dashboard
{
TwistRobot::TwistRobot(int i):
  robot_id(i)
{
  std::stringstream move_topic;
  move_topic << "/X" << robot_id << "/goal";
  move_publisher = nh.advertise<geometry_msgs::Pose>(move_topic.str(), 1 );
}

void TwistRobot::move(double x, double y, double yaw) 
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;

  tf2::Quaternion quat;
  double yaw_rad = yaw * M_PI / 180.0;
  quat.setRPY(0, 0, yaw_rad);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat);
  pose.orientation = quat_msg;
  move_publisher.publish(pose);
}

}