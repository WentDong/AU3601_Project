#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np
from matplotlib import pyplot as plt

x, y, theta = 0, 0, 0 # initial
time_now = 0.

class PID_Controller:
    def __init__(self,kp,ki,kd,output_min,output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
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
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error

        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)

        return self.output

def PoseCallback(pose):
    global x, y, theta
    global time_now

    x = pose.x - 5.544444561 
    y = pose.y - 5.544444561
    theta = pose.theta
    get_time = rospy.Time.now() 
    time_now = get_time.to_sec()

def draw_error(Error, figurename, figurelabel = 1):
    Error[:, 0] -= Error[1, 0]
    traget = np.zeros(shape=(Error.shape[0],))
    plt.figure(figurelabel)
    plt.plot(Error[:, 0], Error[:, 1], label='error', color='blue')
    plt.plot(Error[:, 0], traget, label='target', color='red',linestyle='--')
    plt.xlabel("time(s)")
    plt.ylabel(figurename) 
    plt.legend()

def timeCallback():
    pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size = 10)
    sub = rospy.Subscriber('turtle1/pose', Pose, PoseCallback)
    rate = rospy.Rate(10)
    global time_now

    state = 0
    TransError_x = np.empty(shape=(0, 2)) 
    TransError_y = np.empty(shape=(0, 2))
    RotError_z = np.empty(shape=(0, 2))
    L, H = 3, 2

    cmd_vel = Twist()

    move_er_threshold = 1e-4
    rotate_er_threshold = 1e-3

    move_controller = PID_Controller(kp=9, ki=0, kd = 0, output_min=-0.5, output_max=2)
    rorate_controller = PID_Controller(kp = 9, ki=0, kd= 2, output_min=-0.5, output_max=0.6)

    while not rospy.is_shutdown():
        if state == 0:
            error = L - x
            time_now = rospy.Time.now().to_sec()
            TransError_x = np.append(TransError_x, [[time_now, error]], axis=0)

            if abs(error) < move_er_threshold:
                state += 1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 1:
            error = math.pi / 2 - theta
            RotError_z = np.append(RotError_z, [[time_now, error]], axis=0)

            if abs(error)< rotate_er_threshold:
                state += 1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rorate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rorate_controller.get_output(error)
        elif state == 2:
            error = H - y
            TransError_y = np.append(TransError_y, [[time_now, error]], axis=0)

            if abs(error)< move_er_threshold:
                state += 1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 3:
            if abs(math.pi- theta) > abs(-math.pi - theta):
                error = -math.pi - theta
            else:
                error = math.pi-theta
            # error = min(math.pi - theta, - math.pi - theta) 
            # RotError_z = np.append(RotError_z, [[time_now, error]], axis=0)

            if abs(error)< rotate_er_threshold:
                state += 1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rorate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rorate_controller.get_output(error)
        elif state == 4:
            error = x
            # TransError_x = np.append(TransError_x, [[time_now, error]], axis=0)
            if abs(error)<move_er_threshold:
                state+=1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rorate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 5:
            if abs(math.pi/2 * 3- theta) > abs(-math.pi/2 - theta):
                error = -math.pi/2 - theta
            else:
                error = math.pi/2*3 -theta
            # RotError_z = np.append(RotError_z, [[time_now, error]], axis=0)

            # error = - math.pi /2 - theta
            if abs(error)< rotate_er_threshold:
                state += 1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rorate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rorate_controller.get_output(error)
        elif state == 6:
            error = y 
            # TransError_y = np.append(TransError_y, [[time_now, error]], axis=0)
            if abs(error)<move_er_threshold:
                state+=1
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rorate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 7:
            error = 0 - theta
            # RotError_z = np.append(RotError_z, [[time_now, error]], axis=0)

            if abs(error)< rotate_er_threshold:
                state = 0
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rorate_controller.error = 0
                move_controller.last_error = 0
                move_controller.error_sum = 0
                global theta
                theta -= math.pi * 2
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rorate_controller.get_output(error)
        pub.publish(cmd_vel)
        rospy.loginfo("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s], state: %d, (x,y,theta) = %0.2f, %0.2f %0.2f, Error = %0.2f",
                        cmd_vel.linear.x, cmd_vel.angular.z, state, x,y,theta, error)
        rate.sleep()
    if (TransError_x.size):
        draw_error(TransError_x, "Trans_x", 1)
    if (TransError_y.size):
        draw_error(TransError_y, "Trans_y", 2)
    if (RotError_z.size):
        draw_error(RotError_z, "Rot_z", 3)
    plt.show()

if __name__=="__main__":
    rospy.init_node("Square", anonymous= False)
    timeCallback()


