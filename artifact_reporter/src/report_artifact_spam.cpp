#include "ros/ros.h"

#include "boost/bind.hpp"
#include "boost/ref.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <cmath>

#include "object_detection/DetectedObjectMsg.h"
#include "object_detection/DetectedObjectsMsg.h"
#include "noroute_mesh/send_artifact.h"

#define DETECTIONS_TOPIC "object_detector/detected"
#define ARTIFACT_SRV_NAME "send_artifact"
#define ARTIFACT_SIMILARITY_DIST 1.5 // meters

class ArtifactReport {

private:
    ros::NodeHandle nh;
    ros::Subscriber detection_sub;
    ros::Timer timer;
    ros::ServiceClient artifact_srv_client;
    geometry_msgs::Point artifact_location;
    std::string artifact_type_str;
    bool have_an_artifact_to_report;
public:

    ArtifactReport(ros::NodeHandle nh_) : nh(nh_), have_an_artifact_to_report(false)
    {
        // found artifacts will be attempted to be sent periodically through a timer
        timer = nh.createTimer(ros::Duration(1.0), &ArtifactReport::reportArtifacts, this);

        // wait for h_comms ros service to start reporting
        ROS_WARN("[Artifact-Report-Spam] Waiting for %s to load....", ARTIFACT_SRV_NAME);
        artifact_srv_client = nh.serviceClient<noroute_mesh::send_artifact>(ARTIFACT_SRV_NAME);
        ROS_INFO("[Artifact-Report-Spam] %s ROS service has loaded successfully, ready to send "
                                        "artifact reports", ARTIFACT_SRV_NAME);

        detection_sub = nh.subscribe(DETECTIONS_TOPIC, 1, &ArtifactReport::objectDetectionCb, this);
    }

    void objectDetectionCb(const object_detection::DetectedObjectsMsg& msg)
    {
        // Wait until previous detections have been reported to begin processing new detections
        // if (have_an_artifact_to_report) {
        //     return;
        // }

        have_an_artifact_to_report = true;

        // for now only care about first detection
        auto detection = msg.detected_objects_msgs[0];
        std::string temp(detection.class_name.c_str());

        std::string prev_artifact_type_str = artifact_type_str;
        geometry_msgs::Point prev_artifact_location = artifact_location;

        if (temp == "backpack") {
            artifact_type_str = "TYPE_BACKPACK";
        } else if (temp == "rope") {
            artifact_type_str = "TYPE_ROPE";
        } else if (temp == "helmet") {
            artifact_type_str = "TYPE_HELMET";
        } 
        // Ignore phone as detector buggy
        // else if (temp == "phone") {
        //     artifact_type_str = "TYPE_PHONE";
        // } 
        else if (temp == "survivor") {
            artifact_type_str = "TYPE_RESCUE_RANDY";
        } else {
            ROS_ERROR("[Artifact-Report-Spam] Wrong class of artifact %s received. Discarded.", artifact_type_str.c_str());
            have_an_artifact_to_report = false;
            return;
        }
        
        ROS_INFO("[Artifact-Report-Spam] %s artifact received successfully from object detector", artifact_type_str.c_str());

        artifact_location = detection.global_point.point;

        // dont report nans
        if (isArtifactLocationNan(artifact_location) || isArtifactLocationNan(prev_artifact_location)) {
            ROS_ERROR("[Artifact-Report-Spam] NAN detection. artifact %s Discarded.", artifact_type_str.c_str());
            have_an_artifact_to_report = false;
            return;
        }

        // dont report if previous and current detections are not the "same"
        if (!isArtifactReportSimilar(prev_artifact_type_str, artifact_type_str, prev_artifact_location, artifact_location)) {
            ROS_ERROR("[Artifact-Report-Spam] Not 2 consecutive similar detections. artifact %s Discarded.", artifact_type_str.c_str());
            have_an_artifact_to_report = false;
            return;
        } 

    }

    bool isArtifactLocationNan(geometry_msgs::Point location) {
        return (std::isnan(artifact_location.x) || std::isnan(artifact_location.y) || std::isnan(artifact_location.z));
    }

    bool isArtifactReportSimilar(std::string prev_artifact_type_str, std::string artifact_type_str,
                                geometry_msgs::Point prev_artifact_location, geometry_msgs::Point artifact_location) {
        
        if (prev_artifact_type_str != artifact_type_str) {
            return false;
        }

        if (getDist(prev_artifact_location,artifact_location) > ARTIFACT_SIMILARITY_DIST) {
            return false;
        }

        return true;
    }

    double getDist(geometry_msgs::Point point1, geometry_msgs::Point point2) {
        return std::sqrt(std::pow(point1.x-point2.x, 2) + std::pow(point1.y-point2.y, 2) + std::pow(point1.z-point2.z, 2));
    }

    void reportArtifacts(const ros::TimerEvent &)
    {
        if (!have_an_artifact_to_report) {
            return;
        }

        noroute_mesh::send_artifact srv;

        srv.request.type = artifact_type_str;
        srv.request.x = artifact_location.x;
        srv.request.y = artifact_location.y;
        srv.request.z = artifact_location.z;

        // report the artifact
        if (artifact_srv_client.call(srv)) {
            ROS_INFO("[Artifact-Report-Spam] %s artifact reported successfully at %f, %f, %f", artifact_type_str.c_str(), artifact_location.x,
                                                                                       artifact_location.y,
                                                                                       artifact_location.z);
        }
        else {
            ROS_ERROR("[Artifact-Report-Spam] One of 3 issues occurred while sending artifact report:\n"
                            "1. Unknown Artifact Type Provided\n2. Error in serializing message\n3. Error sending report to base_station");
        }
        

        have_an_artifact_to_report = false;
    }
};




int main (int argc, char** argv)
{
    ros::init(argc, argv, "report_artifact_spam_node");
    ros::NodeHandle nh;

    ArtifactReport artifact_report(nh);    

    ros::spin();
}