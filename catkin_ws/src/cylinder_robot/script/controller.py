#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import math

x, y = 0, 0
time_now = 0

# x_gt, y_gt = 0, 0

state = 0

class P3D_Controller:
    def __init__(self,kp,kdd,kd,output_min,output_max):
        self.kp = kp
        self.kdd = kdd
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_ddiff = 0
        self.last_diff = 0
        self.error_diff = 0
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0

    def constrain(self, output):
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    def get_output(self, error):
        self.error = error
        # self.error_sum += self.error
        self.error_diff = self.error - self.last_error

        self.error_ddiff = self.error_diff - self.last_diff
        self.last_diff = self.error_diff

        self.last_error = self.error

        output = self.kp * self.error + self.kdd * self.error_ddiff + self.kd * self.error_diff
        self.output = self.constrain(output)

        return self.output


def PoseCallback(msg):
    global x, y
    global time_now

    x = msg.pose.position.x
    y = msg.pose.position.y
    time_now = rospy.Time.now().to_sec()


# def GroundTruthCallback(msg):
#     global x_gt, y_gt
#     global time_now

#     x_gt = msg.pose.position.x
#     y_gt = msg.pose.position.y
#     time_now = rospy.Time.now().to_sec()


num_key_points = 8
# Key_Points_x = [4, 4, -4, -4, 0] #should lie in the walls, current is (-5, 5)
# Key_Points_y = [4, -4, 4, -4, 0] 

Key_Points_x = [4, 4,    -2.5, -1.5, -1.5, -4, -4, 0]
Key_Points_y = [4, -2.5, -2.5, -2.5, -4,   -4, 4,  0]

def sign(x):
    if x>=0:
        return 1
    else:
        return -1

def controller():
    pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
    rospy.Subscriber('/robot/esti_model_state', ModelState, PoseCallback)
    # rospy.Subscriber('/robot/set_model_state', ModelState, GroundTruthCallback)
    rospy.init_node('talker',anonymous=True)
    #============design your trace below=============

    global time_now

    global state

    global Key_Points_x
    global Key_Points_y
    global num_key_points
    rate = rospy.Rate(10)

    threshold = 2e-2

    x_controller = P3D_Controller(kp=3.5, kdd=0.04, kd = 180, output_min=-200, output_max=200)
    y_controller = P3D_Controller(kp=3.5, kdd=0.04, kd = 180, output_min=-200, output_max=200)

    cmd_force = Twist()
    cmd_force.linear.x = 0
    cmd_force.angular.z = 0
    # while not rospy.is_shutdown():
    while state<num_key_points:
        # error_x = math.clip(Key_Points_x[state]-x, (-0.2, 0.2))
        # error_y = math.clip(Key_Points_y[state]-y, (-0.2, 0.2))
        error_x = Key_Points_x[state]-x
        error_y = Key_Points_y[state]-y
        # error_x = min(abs(Key_Points_x[state]-x),4) * sign(Key_Points_x[state] - x)
        # error_y = min(abs(Key_Points_y[state]-y), 4) * sign(Key_Points_y[state] - y)
        # if (abs(Key_Points_x[state]-x)>4 and abs(Key_Points_y[state] - y)>4):
            # p = abs(Key_Points_x[state] - x) / abs(Key_Points_y[state]-y)
        # else:
            # p = 1

        # if p>1:
            # error_y /= p
        # else:
            # error_x *= p
        
        if abs(error_x) < threshold and abs(error_y)<threshold:
            state += 1
            # state %= num_key_points

            cmd_force.linear.x = 0
            cmd_force.angular.z = 0
            x_controller.error = 0
            x_controller.last_error = 0
            x_controller.last_diff = 0

            y_controller.error_sum = 0
            y_controller.error = 0
            y_controller.last_diff = 0
        else:
            cmd_force.linear.x = x_controller.get_output(error_x)
            cmd_force.angular.z = y_controller.get_output(error_y)
        
        pub.publish(cmd_force)
        rospy.loginfo("Publish Robot Force command[%0.2f N, %0.2f N], measured state: %d, (x,y) = %0.2f, %0.2f, Error_x = %0.2f, Error_y = %0.2f",
                        cmd_force.linear.x, cmd_force.angular.z, state, x,y, error_x, error_y)
        rate.sleep()


    # for i in range(0,100):
    #     twist = Twist()
    #     twist.linear.x=1.5*abs(i-49.5)/(i-49.5)
    #     twist.angular.z=0.5*abs(i-49.5)/(i-49.5)
    #     pub.publish(twist)
    #     rate.sleep()
    # sys.exit(0)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass


