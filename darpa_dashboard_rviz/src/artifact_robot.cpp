#include <rviz/panel.h>

#include "artifact_robot.h"
#include "std_msgs/String.h"
#include "interface_protocol/PostReport.h"
#include "interface_protocol/PostReportRequest.h"
#include "interface_protocol/PostReportResponse.h"

namespace darpa_dashboard
{
ArtifactRobot::ArtifactRobot(int i):
  robot_id(i)
{
  std::stringstream drop_artifact_topic;
  drop_artifact_topic << "/X" << robot_id << "/report";
  drop_artifact_service_client = nh.serviceClient<interface_protocol::PostReport>("post_report");
}

void ArtifactRobot::drop(const double x, const double y, const double z, const std::string artifact) 
{
  interface_protocol::PostReportRequest post_report_request;
  interface_protocol::PostReportResponse post_report_response;

  post_report_request.x = x;
  post_report_request.y = y;
  post_report_request.z = z;
  post_report_request.type = artifact;

  drop_artifact_service_client.call(post_report_request, post_report_response);

  ROS_INFO_STREAM(post_report_response.report_status);
}

}