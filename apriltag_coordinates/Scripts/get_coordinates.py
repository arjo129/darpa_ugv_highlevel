#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = rospy.Publisher("/start_ref_apriltag", AprilTagDetection, queue_size=10)

def callback(data):
    if len(data.detections) == 2:
        head = data.header

        first, second = data.detections[0],data.detections[1]
        i_first = first.pose.pose.pose.position
        i_second = second.pose.pose.pose.position
        #print("First: ", i_first)
        #print("Second: ", i_second)

        result = [0,0,0]
        result[0] = (i_first.x + i_second.x) / 2
        result[1] = (i_first.y + i_second.y) / 2
        result[2] = (i_first.z + i_second.z) / 2

        new_msg = AprilTagDetection()
        new_msg.id = [1000]
        new_msg.size = [0.16]

        geo = PoseWithCovarianceStamped()
        geo.header = head
        geo.pose.pose.position.x = result[0]
        geo.pose.pose.position.y = result[1]
        geo.pose.pose.position.z = result[2]
        
        new_msg.pose = geo
        pub.publish(new_msg)

        print("Between: ", result)

    
def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()