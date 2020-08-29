#include "ros/ros.h"

#include "boost/bind.hpp"
#include "boost/ref.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ignition/msgs.hh"
#include "subt_communication_broker/subt_communication_client.h"
#include "subt_ign/CommonTypes.hh"
#include "subt_ign/protobuf/artifact.pb.h"

#include "object_detection/DetectedObjectMsg.h"
#include "object_detection/DetectedObjectsMsg.h"

#define ROBOT_NAME "X1"
#define DETECTIONS_TOPIC "/object_detector/detected"

class ArtifactReport {

private:
    ros::NodeHandle nh;
    ros::Subscriber detection_sub;
    ros::Timer timer;
    subt::CommsClient comms_client;
    geometry_msgs::Point artifact_location;
    subt::ArtifactType artifact_type;
    std::string artifact_type_str;
    bool have_an_artifact_to_report;
public:

    ArtifactReport(ros::NodeHandle nh_) : nh(nh_), comms_client(ROBOT_NAME), have_an_artifact_to_report(false)
    {
        // found artifacts will be attempted to be sent periodically through a timer
        timer = nh.createTimer(ros::Duration(1.0), &ArtifactReport::reportArtifacts, this);

        // set up communications with the base station for artifact reporting
        // comms_client.Bind(&BaseStationCallback, ROBOT_NAME);

        detection_sub = nh.subscribe(DETECTIONS_TOPIC, 1, &ArtifactReport::objectDetectionCb, this);
    }

    void objectDetectionCb(const object_detection::DetectedObjectsMsg& msg)
    {
        have_an_artifact_to_report = true;

        // for now only care about first detection
        auto detection = msg.detected_objects_msgs[0];
        std::string temp(detection.class_name.c_str());
        artifact_type_str = temp;

        if (artifact_type_str == "backpack") {
            artifact_type = subt::ArtifactType::TYPE_BACKPACK;
        } else if (artifact_type_str == "rope") {
            artifact_type = subt::ArtifactType::TYPE_ROPE;
        } else if (artifact_type_str == "helmet") {
            artifact_type = subt::ArtifactType::TYPE_HELMET;
        } else if (artifact_type_str == "phone") {
            artifact_type = subt::ArtifactType::TYPE_PHONE;
        } else if (artifact_type_str == "survivor") {
            artifact_type = subt::ArtifactType::TYPE_RESCUE_RANDY;
        } else {
            ROS_ERROR("[Artifact-Report-Spam] Wrong class of artifact %s received. Discarded.", artifact_type_str.c_str());
            have_an_artifact_to_report = false;
            return;
        }
        
        ROS_INFO("[Artifact-Report-Spam] %s artifact received successfully from object detector", artifact_type_str.c_str());

        artifact_location = detection.global_point.point;

    }

    void reportArtifacts(const ros::TimerEvent &)
    {
        if (!have_an_artifact_to_report) {
            return;
        }

        ignition::msgs::Pose pose;
        pose.mutable_position()->set_x(artifact_location.x);
        pose.mutable_position()->set_y(artifact_location.y);
        pose.mutable_position()->set_z(artifact_location.z);

        // fill the type and pose
        subt::msgs::Artifact artifact;
        artifact.set_type(static_cast<uint32_t>(artifact_type));
        artifact.mutable_pose()->CopyFrom(pose);

        // serialize the artifact
        std::string serializedData;
        if (!artifact.SerializeToString(&serializedData)) {
            ROS_ERROR_STREAM("ArtifactReporter::ReportArtifact(): Error serializing message\n" << artifact.DebugString());
        }

        // report the artifact
        comms_client.SendTo(serializedData, subt::kBaseStationName);
        ROS_INFO("[Artifact-Report-Spam] %s artifact reported successfully at %f, %f, %f", artifact_type_str.c_str(), artifact_location.x,
                                                                                       artifact_location.y,
                                                                                       artifact_location.z);

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