#!/usr/bin/env python3

import rospy
from interface_protocol.srv import MappingUpdate

def test_map_update():
    try:
        post = rospy.ServiceProxy('post_map_update', MappingUpdate)
        resp = post()

        s = resp.result.data
        print(s)

    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def test_telemetry_update():
    try:
        post = rospy.ServiceProxy('post_telemetry_update', MappingUpdate)
        resp = post()

        s = resp.result.data
        print(s)
        
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def test_markers_update():
    try:
        post = rospy.ServiceProxy('post_markers_update', MappingUpdate)
        resp = post()

        s = resp.result.data
        print(s)
        
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.wait_for_service('post_map_update')
    test_map_update()
    rospy.wait_for_service('post_telemetry_update')
    test_telemetry_update()
    rospy.wait_for_service('post_markers_update')
    test_markers_update()