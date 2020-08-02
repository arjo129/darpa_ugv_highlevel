#ifndef ARTIFACT_ROBOT_H
#define ARTIFACT_ROBOT_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

namespace darpa_dashboard
{

class ArtifactRobot
{
public:
  ArtifactRobot(int);
  void drop(const double, const double, const double, const std::string);

private:
  int robot_id;

  ros::ServiceClient drop_artifact_service_client;

  ros::NodeHandle nh;
};

}

#endif 
