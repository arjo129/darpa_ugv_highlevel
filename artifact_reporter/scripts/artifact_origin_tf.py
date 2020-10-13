#!/usr/bin/env python

import rospy
import tf2_ros
from subt_msgs.srv import PoseFromArtifact, PoseFromArtifactRequest
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String


SUBT_SRV_NAME = "/subt/pose_from_artifact_origin"

def send_transform(pose, parent_frame, child_frame):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent_frame
    static_transformStamped.child_frame_id = child_frame

    static_transformStamped.transform.translation.x = pose.pose.pose.position.x
    static_transformStamped.transform.translation.y = pose.pose.pose.position.y
    static_transformStamped.transform.translation.z = pose.pose.pose.position.z

    static_transformStamped.transform.rotation.x = pose.pose.pose.orientation.x
    static_transformStamped.transform.rotation.y = pose.pose.pose.orientation.y
    static_transformStamped.transform.rotation.z = pose.pose.pose.orientation.z
    static_transformStamped.transform.rotation.w = pose.pose.pose.orientation.w

    broadcaster.sendTransform(static_transformStamped)

    rospy.loginfo("[ARTIFACT-ORIGIN-TF] Static Artifact Origin TRANSFORM Sent")

    rospy.spin()

def main():
    robot_name = None
    robot_name = rospy.get_param("~robot_name")

    rospy.logwarn("[ARTIFACT-ORIGIN-TF] Waiting for darpa artifact origin service....")
    rospy.wait_for_service(SUBT_SRV_NAME)
    rospy.loginfo("[ARTIFACT-ORIGIN-TF] Subt Artifact Origin Service is active")

    artifact_origin_pose = PoseStamped()

    try:
        get_artifact_origin = rospy.ServiceProxy(SUBT_SRV_NAME, PoseFromArtifact)
        robot_name_msg = String()
        robot_name_msg.data = robot_name
        artifact_origin_pose = get_artifact_origin(robot_name_msg)
    except rospy.ServiceException as e:
        rospy.logerr("[ARTIFACT-ORIGIN-TF] Service call failed: %s"%e)
        rospy.logerr("[ARTIFACT-ORIGIN-TF] Likely robot too far from staging area. Please restart sim...")
        return

    rospy.loginfo("[ARTIFACT-ORIGIN-TF] Artifact Origin Pose Received...\n{}".format(artifact_origin_pose))

    send_transform(artifact_origin_pose, "artifact_origin", robot_name+"/world")




if __name__ == "__main__":
    rospy.init_node('artifact_origin_tf_node')
    main()

