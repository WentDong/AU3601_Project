#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def Initialize():
    rospy.init_node('velocity_publisher', anonymous= False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)

   
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.4

        pub.publish(vel_msg)
        rospy.loginfo("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]",
                        vel_msg.linear.x, vel_msg.angular.z)
        rate.sleep()

if __name__ == "__main__":
    try:
        Initialize()
    except rospy.ROSInterruptException:
        pass