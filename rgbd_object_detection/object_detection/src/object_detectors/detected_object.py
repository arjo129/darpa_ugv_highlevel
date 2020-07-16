#!/usr/bin/env python

from utils import utils

class DetectedObject:
    # cv2_bbox follows cv2 (left, top, width, height) format
    # centered_bbox following (center_x, center_y, width, height) format. | y-axis, --- x-axis
    def __init__(self, class_name, confidence, cv2_bbox):

        self.class_name = class_name
        self.confidence = confidence
        self.cv2_bbox = cv2_bbox
        self.width = int(cv2_bbox[2])
        self.height = int(cv2_bbox[3])

        self.cv2_rect = self.get_cv2_rect()
        self.centered_bbox = self.get_centered_bbox()
        self.center = self.centered_bbox[0:2] # center_x, center_y

    def get_centered_bbox(self):
        cv2_bbox = self.cv2_bbox
        left, top = cv2_bbox[0], cv2_bbox[1]
        width, height = cv2_bbox[2], cv2_bbox[3]
        centered_bbox = [left + width/2, top + height/2, width, height]

        # ensure all values are integer
        centered_bbox = [int(pos) for pos in centered_bbox]

        return centered_bbox


    # cv2 rectangle bbox follows (top left), (bottom right) format
    def get_cv2_rect(self):
        bbox = self.cv2_bbox
        return (bbox[0], bbox[1]), (bbox[0]+self.width, bbox[1]+self.height)

    def set_xyz(self, xyz_coord):
        if (xyz_coord == []):
            self.xyz_coord = (0,0,0)
        else:
            self.xyz_coord = xyz_coord[0]


    def __str__(self):
        xyz_coord = ["{:.2f}".format(pos) for pos in self.xyz_coord]
        obj_str = "{} detected with {:.1f}% confidence at {} location".format(self.class_name, 100.0*self.confidence, xyz_coord)
        return obj_str