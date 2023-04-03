#!/usr/bin/env python

import rospy
from tutorials.msg import person
def talker():
    pub = rospy.Publisher('people', \
        person, queue_size = 10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = person()
        msg.name = "Ice"
        msg.age = 18
        rospy.loginfo("send Name=Ice, age=18.")
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass