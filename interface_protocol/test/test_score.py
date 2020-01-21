#!/usr/bin/env python3

import rospy
import json
import pprint
from interface_protocol.srv import GetStatus,PostReport

def test_get_status():
    try:
        get = rospy.ServiceProxy('get_status', GetStatus)
        resp = get()

        s = resp.result.data

        # Change the quotes to double quotes for JSON library to accept the string
        s = s.replace("\'","\"")
        print(s)
        j = json.loads(s)
        pprint.pprint(j)
        
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def test_post_report():
    try:
        post = rospy.ServiceProxy('post_report', PostReport)

        # Good test - should show tahtt he score change is 1 in the test server
        resp = post(1,2,3,"backpack")

        s = resp.result.data
        s = s.replace("\'","\"")
        print(s)
        j = json.loads(s)
        pprint.pprint(j)
        
        # Bad test - should show that the score change is 0 in the test server
        resp = post(1,2,3,"b")

        s = resp.result.data
        s = s.replace("\'","\"")
        print(s)
        j = json.loads(s)
        pprint.pprint(j)

    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.wait_for_service('get_status')
    test_get_status()
    rospy.wait_for_service('post_report')
    test_post_report()