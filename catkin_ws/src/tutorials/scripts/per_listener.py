#!/usr/bin/env python

import rospy
from tutorials.msg import person

def callback(data):
    rospy.loginfo(rospy.get_caller_id() +\
        "heard Name=%s" %data.name)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('name', person, callback)
    rospy.spin()

if __name__=="__main__":
    listener()