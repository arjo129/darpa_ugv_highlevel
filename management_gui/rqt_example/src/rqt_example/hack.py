#!/usr/bin/python
import rospy
import tf
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np


rospy.init_node("fake_detector")
pub = rospy.Publisher("/artifact", String)
artifact_vis = rospy.Publisher("/artifact/visualization", MarkerArray)
markers = MarkerArray()
listener = tf.TransformListener()
prev_pos = None
distance_covered = 0
index = 0
increments = [3, 7.8, 17.8]
objects = ["Manikin (0.8, -7, 1.3)", "Extinguisher (2, -12.5, 1.1)", "Backpack (3.8, -7.9, 0.2)"]
coordinates = [(0.8, -7, 1.3), (2, -12.5, 1.1), (3.8, -7.9, 0.2)]
while not rospy.is_shutdown():
    try:
        now = rospy.Time.now()
        listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform("map", "base_link", now)
        pos = trans
        if not prev_pos:
            prev_pos = pos
        distance_covered += ((pos[0]-prev_pos[0])**2 +( pos[1]-prev_pos[1])**2)**0.5
        print(distance_covered)
        if distance_covered > increments[index]:
            pub.publish(objects[index])
            obj_marker = Marker()
            obj_marker.header.frame_id = "map"
            obj_marker.id = index
            obj_marker.pose.position.x = coordinates[index][0]
            obj_marker.pose.position.y = coordinates[index][1]
            obj_marker.pose.position.z = coordinates[index][2]
            obj_marker.color.a = 1
            obj_marker.scale.z = 0.23
            obj_marker.type = 9
            obj_marker.text = objects[index]
            markers.markers.append(obj_marker)
            artifact_vis.publish(markers)
            index += 1
        prev_pos = pos
    except Exception as E:
        print(E)
        print ("Whoops no base_link transform")