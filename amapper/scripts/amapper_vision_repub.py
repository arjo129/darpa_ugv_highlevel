#!/usr/bin/env python

"""
Stop gap measure to allow amapper objects to be used in vision pipeline.
"""

from vision.msg import DetectedObjects, DetectedObject
from asv_msgs.msg import AmapperObjectlist
from geometry_msgs.msg import PointStamped
from math import atan2
import rospy
import tf
import numpy as np

LIDAR_FRAME = "puck_joint_front_link"

def point_to_np(point):
    return np.array([point.x, point.y, 0])

def distance_between_points(pt1, pt2):
    return ((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)**0.5

class AmapperAdapter:
    def __init__(self):
        self.transform_listener = tf.TransformListener()
        self.sub = rospy.Subscriber("/asv/amapper_object_list", AmapperObjectlist, self.on_recieve_blob)
        self.pub = rospy.Publisher("/asv/amapper_vision_adapter/objects", DetectedObjects)

    def on_recieve_blob(self, blob):
        object_list = []
        for blob in blob.objects:
            pt = PointStamped()
            pt.header = blob.header
            pt.point = blob.centroid
            self.transform_listener.waitForTransform(LIDAR_FRAME, pt.header.frame_id, pt.header.stamp, rospy.Duration(5.0))
            pt = self.transform_listener.transformPoint(LIDAR_FRAME, pt)
            obj = DetectedObject()
            obj.name = "amapper_blob"
            obj.real_coords =[pt.point.x, pt.point.y, 0]
            if len(blob.mapContour) > 0:
                obj.real_dims = [distance_between_points(blob.mapContour[3],blob.mapContour[2]), distance_between_points(blob.mapContour[1], blob.mapContour[2]), 5]
            elif blob.area < 5:
                obj.real_dims = [1,1,5]
            else:
                continue

            if len(obj.real_dims) > 1 and len(blob.mapContour) >3:
                if obj.real_dims[0] < obj.real_dims[1]:
                    point_in_lidar_frame = PointStamped()
                    point_in_lidar_frame.header = blob.header
                    point_in_lidar_frame.point = blob.mapContour[3]
                    point_in_lidar_frame = self.transform_listener.transformPoint(LIDAR_FRAME, point_in_lidar_frame)
                    p1 = point_to_np(point_in_lidar_frame.point)
                    point_in_lidar_frame = PointStamped()
                    point_in_lidar_frame.header = blob.header
                    point_in_lidar_frame.point=  blob.mapContour[2]
                    point_in_lidar_frame = self.transform_listener.transformPoint(LIDAR_FRAME, point_in_lidar_frame)
                    p2 = point_to_np(point_in_lidar_frame.point)
                    if np.linalg.norm(p1) > np.linalg.norm(p2):
                        result = p2-p1
                        angle = atan2(result[1], result[0])
                    else:
                        result = p1-p2
                        angle = atan2(result[1], result[0])
                    obj.angle = angle 
                else:
                    point_in_lidar_frame = PointStamped()
                    point_in_lidar_frame.header = blob.header
                    point_in_lidar_frame.point = blob.mapContour[1]
                    point_in_lidar_frame = self.transform_listener.transformPoint(LIDAR_FRAME, point_in_lidar_frame)
                    p1 = point_to_np(point_in_lidar_frame.point)
                    point_in_lidar_frame = PointStamped()
                    point_in_lidar_frame.header = blob.header
                    point_in_lidar_frame.point = blob.mapContour[2]
                    point_in_lidar_frame = self.transform_listener.transformPoint(LIDAR_FRAME, point_in_lidar_frame)
                    p2 = point_to_np(point_in_lidar_frame.point)
                    if np.linalg.norm(p1) > np.linalg.norm(p2):
                        result = p2-p1
                        angle = atan2(result[1], result[0])
                    else:
                        result = p1-p2
                        angle = atan2(result[1], result[0])
                    obj.angle = angle 
            
            obj.area = blob.area
            object_list.append(obj)
        detected_list = DetectedObjects()
        detected_list.detected = object_list
        self.pub.publish(detected_list)


rospy.init_node("amapper_vision_republisher")
amapper_adapter = AmapperAdapter()
rospy.spin()
del amapper_adapter