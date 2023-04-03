#!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as lnr
from scipy import integrate
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from scipy.linalg import expm
from gazebo_msgs.msg import ModelState
import os
import sys
# ===============your figs' path ==============
path_fig = "/dssg/home/acct-stu/stu1418/catkin_ws/src/cylinder_robot/figs"
#rospkg.RosPack().get_path('cylinder_robot')

class KalmanFilter(object):
    ##################################### initializing ####################################
    def __init__(self):
        self.start_time = rospy.get_time()
        self.last_time = rospy.get_time()
        # self.N = 0      #sample times
        self.n = 4      #dimension of x
        self.p = 2      #dimension of y
        self.mass = 15     #supposed mass for robot

        self.u = np.zeros([2,1])     #control signal
        self.u_last = np.zeros([2,1])       #control in last time
        
        # data to be stored
        self.x_predicted = None     #x after prediction
        self.P_predicted = None
        self.x_updated = np.zeros([4,1])        # x after update. initialize with [0 0 0 0]T
        self.P_updated = np.eye(4)      # P after update. initialize with ones
        self.x_last = self.x_updated
        self.P_last = self.P_updated

        # continuous case
        self.A_tilde = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0],[0, 0, 0, 0]])        #A in contineous case
        self.B_tilde = np.array([[0., 0.], [0., 0.], [1/self.mass, 0.], [0., 1/self.mass]])     #B in contineous case
        self.C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])     # observe matrix
        self.Q_tilde = 0.001 * np.array([[0.05, 0.0, 0.0, 0.0],[0.0, 0.05, 0.0, 0.0],[0.0, 0.0, 0.05, 0.0],[0.0, 0.0, 0.0, 0.05]])
        self.R = np.array([[0.008**2, 0],[0, 0.008**2]])

        # discrete case
        self.A = None
        self.B = None
        self.Q = None

        # for plot, data restore
        self.plot_N=100     #the number of sample points for visualization
        self.x_real_store = []      #list to store the real Modelstate
        self.x_filter_store = []        #list to store the Modelstate filtered
        self.x_measure_store = []       #list to store the measured Modelstate
        self.innovation_store=[]       #list to store the innovation

        # config the subscribe information
        rospy.Subscriber('/robot/control', Twist, self.callback_twist)
        rospy.Subscriber('/robot/observe', LaserScan, self.callback_laserscan)
        rospy.Subscriber('gazebo/set_model_state', ModelState, self.callback_state)
        self.pub = rospy.Publisher('/robot/esti_model_state', ModelState, queue_size = 10)
        rospy.on_shutdown(self.visualization)

    ## ===================================== Edit bellow =====================================
    ## ==========================================================================   vvvvvvvvvv
    ################################### prediction_step ###################################
    def prediction_step(self, u):
        # print(self.A, self.x_last, self.B, u)
        self.x_predicted = self.A.dot(self.x_last) + self.B.dot(u) #eq1
        self.P_predicted = self.A.dot(self.P_last).dot(self.A.T) + self.Q #eq2
        self.x_last = self.x_predicted
        self.P_last = self.P_predicted
        # print('x_pre', self.x_predicted)

    ##################################### update_step #####################################
    def update_step(self, y):
        innovation = y - self.C.dot(self.x_last)
        kalman_gain = self.P_last.dot(self.C.T).dot(np.linalg.inv(self.C.dot(self.P_last).dot(self.C.T)+self.R) ) #eq3
        lambda_t = kalman_gain .dot(self.C)
        self.x_updated = self.x_last + kalman_gain.dot(innovation)
        self.P_updated = self.P_last - lambda_t.dot(self.P_last)  #eq5
        self.innovation_store.append((rospy.get_time(),innovation[0][0],innovation[1][0]))      #for plot
        self.x_last = self.x_updated
        self.P_last = self.P_updated
        # print('x_up', self.x_updated)
    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================
    ############################### calculate A and B with dt ###############################
    def discretization(self, dt):
        n, m = self.B_tilde.shape
        tmp = np.hstack((self.A_tilde, self.B_tilde))
        tmp = np.vstack((tmp,np.zeros((m, n + m))))
        tmp = lnr.expm(tmp*dt)
        self.A = tmp[:n, :n]
        self.B = tmp[:n, n:n+m]
        
        q_1, q_2, q_3, q_4, q_5 = self.Q_tilde[0, 0], self.Q_tilde[0, 1], self.Q_tilde[0, 2], self.Q_tilde[0, 3], self.Q_tilde[1, 1]
        q_6, q_7, q_8, q_9, q_10 = self.Q_tilde[1, 2], self.Q_tilde[1, 3], self.Q_tilde[2, 2], self.Q_tilde[2, 3], self.Q_tilde[3, 3]
        self.Q = np.array([[q_1*dt+q_3*dt**2, q_2*dt+0.5*q_4*dt**2+0.5*q_6*dt**2, q_3*dt+0.5*q_8*dt**2, q_4*dt+0.5*q_9*dt**2],\
            [0.5*q_2*dt+q_4*dt**2+0.5*q_6*dt**2, q_5*dt+q_7*dt**2, q_6*dt+0.5*q_9*dt**2, q_7*dt+0.5*q_10*dt**2],\
            [q_3*dt+0.5*q_8*dt**2, q_6*dt+0.5*q_9*dt**2, q_8*dt, q_9*dt],\
            [q_4*dt+0.5*q_9*dt**2, q_7*dt+0.5*q_10*dt**2, q_9*dt, q_10*dt]])
    ## ===================================== Edit bellow =====================================
    ## ==========================================================================   vvvvvvvvvv
    ########################### callback function of control signal ##########################
    def callback_twist(self,Twist):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.discretization(dt)
        ### TODO: HERE is Control time c_t, do prediction step with old u, and update u
        self.u_last = self.u
        self.u = np.array([Twist.linear.x, Twist.angular.z]).reshape((2,1))
        # print("TWIST:", self.u)
        self.prediction_step(self.u_last)
        # ...
        # self.u_last = ...
        ##added content
        # ...
        # ...
        self.last_time = current_time

    ########################### callback function of laser ##################################
    def callback_laserscan(self,LaserScan):
        current_time = rospy.get_time()
        dt  = current_time - self.last_time
        self.discretization(dt)

        # print(len(LaserScan.ranges),LaserScan.ranges)
        # x = 5 - LaserScan.ranges[0]
        # y = 5 - LaserScan.ranges[1]
        eps = 1e-1
        Lx = LaserScan.ranges[0] + LaserScan.ranges[2]
        Dx = LaserScan.ranges[2] - LaserScan.ranges[0]

        Ly = LaserScan.ranges[1] + LaserScan.ranges[3]
        Dy = LaserScan.ranges[3] - LaserScan.ranges[1]

        if (abs(Lx - 10)<eps):
            x = Dx/2
        elif (abs(Lx - 6.8)<eps):
            x = (3.2 + Dx)/2
        elif (abs(Lx - 1) < eps):
            x = (-5 + Dx)/2
        elif (abs(Lx - 8)<eps):
            x = (2 + Dx) /2
        elif (abs(Lx - 1.8)<eps):
            x = (-8.2 + Dx)/2
        else:
            x = (Dx)/2
            print("OUT OF CASES! SOMETHING GOES WRONG!")

        if (abs(Ly- 10) < eps):
            y = (Dy)/2
        elif (abs(Ly - 6.8)<eps):
            y = (3.2 + Dy)/2
        elif (abs(Ly - 1) < eps or Ly < 0.6+eps):
            y = (-5 + Dy)/2
        elif (abs(Ly - 1.8) <eps):
            y = (-8.2 + Dy)/2
        else:
            y = (Dy)/2
            print("OUT OF CASES! SOMETHING GOES WRONG!")

        # print("POS FROM LASER:", x,y)

        self.prediction_step(self.u)
        self.update_step(np.array([x,y]).reshape(2,1))

        # ...

        # restore data for final plot
        self.x_filter_store.append( (current_time, self.x_updated[0], self.x_updated[1]))
        self.x_measure_store.append((current_time, x, y))
        self.sendStateMsg()

        # print(current_time - self.start_time, self.N)
        # self.N += 1
        # print(self.N, self.plot_N)
        # if self.N%self.plot_N==0 and self.N/self.plot_N>0:
        #     self.visualization(self.N/self.plot_N)
        # print('visualization complete. ctrl+C to exit')

    def sendStateMsg(self):
        msg = ModelState()
        msg.pose.position.x = self.x_last[0]
        msg.pose.position.y = self.x_last[1]
        self.pub.publish(msg)

    #################################### get real state ####################################
    # get the real state of robot. for plot only. in the real world this cannot be obtained
    def callback_state(self,ModelState):
        current_time = rospy.get_time()
        self.x_real_store.append((current_time, ModelState.pose.position.x, ModelState.pose.position.y))
    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================
    ###################################### visualize #######################################
    def visualization(self):
        
        rospy.loginfo("visualizing")

        x_filtered = np.array(self.x_filter_store)[:, 1:]  # column 0 is time. 1,2 is x,y respectively
        x_filtered_time = np.array(self.x_filter_store)[:, 0]  # time

        x_true = np.array(self.x_real_store)[:, 1:]
        x_true_time = np.array(self.x_real_store)[:, 0]

        x_measure = np.array(self.x_measure_store)[:, 1:]
        x_measure_time = np.array(self.x_measure_store)[:, 0]

        inno_x=np.array(self.innovation_store)[:, 1]
        inno_y=np.array(self.innovation_store)[:, 2]
        inno_time=np.array(self.innovation_store)[:, 0]

        fig1 = plt.figure(figsize=(20,9))
        ax = fig1.subplots(1,2)

        x_e, =ax[0].plot(x_filtered_time, x_filtered[:, 0], label='x_estimate')  # x_filtered[:, 0] is x
        x_t, =ax[0].plot(x_true_time, x_true[:, 0], label='x_true')
        y_e, =ax[1].plot(x_filtered_time, x_filtered[:, 1], label='y_estimate')
        y_t, =ax[1].plot(x_true_time, x_true[:, 1], label='y_true')
        x_m, =ax[0].plot(x_measure_time, x_measure[:, 0], label='x_measure')
        y_m, =ax[1].plot(x_measure_time, x_measure[:, 1], label='y_measure')
        ax[0].set_title("Comparison of position x, (measured, estimated result and ground truth)")
        ax[1].set_title("Comparison of position y, (measured, estimated result and ground truth)")
        ax[0].legend(bbox_to_anchor = (0.85,1), loc='upper left')
        ax[1].legend(bbox_to_anchor = (0.85,1), loc='upper left')

        fig1.suptitle("Comparison of position x y")
        # fig1.legend([x_e,x_t, x_m, y_e, y_t, y_m],['x_estimate','x_true','x_measure', 'y_estimate', 'y_true', 'y_measure'])
        fig1.savefig(path_fig+"/position"+".png",dpi = 600)

        plt.figure(2)
        t_e, =plt.plot(x_filtered[:, 0], x_filtered[:, 1], label='trace_filtered') 
        t_t, =plt.plot(x_true[:, 0], x_true[:, 1], label='trace_true')
        t_m, = plt.plot(x_measure[:, 0],x_measure[:, 1] ,label='trace_measure')
        plt.title("Comparison of trace (measured, filtered result and ground truth)")
        plt.legend([t_e,t_t,t_m],['trace_filtered','trace_true','trace_measure'])
        plt.savefig(path_fig+"/trace"+".png",dpi = 600)

        # plt.figure(3)
       
        # plt.title("Measurement result of position")
        # plt.legend()
        # plt.savefig(path_fig+"/measure_"+str(N)+".png",dpi = 600)

        # plt.figure(4)
        # vx_e, =plt.plot(x_filtered_time, x_filtered[:, 2], label='vx_estimate')  
        # vx_t, =plt.plot(x_true_time, x_true[:, 2], label='vx_true')
        # plt.title("Comparison of velocity x ( estimated result and ground truth)")
        # plt.legend([vx_e,vx_t],['vx_estimate','vx_true'])
        # plt.savefig(path_fig+"/velocity_x_"+str(N)+".png",dpi = 600)

        # plt.figure(5)
        # vy_e, =plt.plot(x_filtered_time, x_filtered[:, 3], label='vy_estimate')  
        # vy_t, =plt.plot(x_true_time, x_true[:, 3], label='vy_true')
        # plt.title("Comparison of velocity y ( estimated result and ground truth)")
        # plt.legend([vy_e,vy_t],['vy_estimate','vy_true'])
        # plt.savefig(path_fig+"/velocity_y_"+str(N)+".png",dpi = 600)
        fig3 = plt.figure(figsize=(20,9))
        ax3 = fig3.subplots(1,2)
        # plt.figure(3)
        innox, =ax3[0].plot(inno_time, inno_x)  
        innoy, =ax3[1].plot(inno_time, inno_y)  
        fig3.suptitle("Innovation")
        # fig3.legend([innox,innoy],['innovation_x','innovation_y'])
        ax[1].legend(bbox_to_anchor = (0.85,1), loc='upper left')
        ax[0].legend(bbox_to_anchor = (0.85,1), loc='upper left')

        fig3.savefig(path_fig+"/innovation"+".png",dpi = 600)

if __name__ == '__main__':
    try:
        rospy.init_node('kalman_filter', anonymous=True)
        kf = KalmanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
