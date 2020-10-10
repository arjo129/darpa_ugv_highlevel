#!/usr/bin/env python2
# license: http://opencv.org/license.html

import rospy
import sys
import numpy as np
import os.path
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from actionlib import SimpleActionClient
from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal

from object_detectors.detected_object import DetectedObject
from utils import utils


class DarknetObjectDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.detected_objects = []

        self.yolo_client = SimpleActionClient('darknet_ros/check_for_objects', CheckForObjectsAction)
        rospy.logwarn("Waiting for darket_ros server...")
        self.yolo_client.wait_for_server()
        rospy.loginfo("darknet_ros server is up and running...")


    # frame is opencv mat frame, return list of DetectedObject
    def detect(self, frame):
        # Clear Previous Detection
        self.detected_objects = []

        # Create sensor_msg/Image to send to darknet_ros action server
        image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        detection_goal = CheckForObjectsGoal()
        detection_goal.id = 0 # no idea what its for
        detection_goal.image = image_msg

        # Send the image to darknet_ros action server
        self.yolo_client.send_goal(detection_goal)
        self.yolo_client.wait_for_result()
        result = self.yolo_client.get_result()
        detected_objects = result.bounding_boxes.bounding_boxes
        
        # Check the list of objects returned by darknet_ros action server
        for detected_object in detected_objects:
            class_name = detected_object.Class
            confidence = detected_object.probability
            bbox = [detected_object.xmin, 
                    detected_object.ymin, 
                    detected_object.xmax - detected_object.xmin, 
                    detected_object.ymax - detected_object.ymin]

            self.detected_objects.append(DetectedObject(class_name, confidence, bbox))

        return self.detected_objects

