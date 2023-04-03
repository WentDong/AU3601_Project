#!/usr/bin/env python
import numpy as np
from scipy import linalg as lnr
from matplotlib import pyplot as plt
# import matplotlib

class KalmanFilter(object):
    # initialization the kalman filter. 
    #   x'(t) = Ax(t) + Bu(t) + w(t)
    #   y(t) = Cx(t) + v(t)
    #   x(0) ~ N(x_0, P_0)
    def __init__(self, mass, C, Sigma_w, Sigma_v, x_0, P_0):
        self.mass = mass
        self.C = C
    
        self.n = 4
        self.m = 2

        self.Sigma_w = Sigma_w
        self.Sigma_v = Sigma_v
        
        self.t = 0
        self.x = x_0
        self.P = P_0
        self.u = np.zeros([self.m, 1])

    # Given duration dt, return the discretization of A, B, Sigma_w. Just like what we do last week.
    def _discretization_Func(self, dt):
        Atilde = np.array([
            [1, 0,  0,  0],
            [dt,1,  0,  0],
            [0, 0,  1,  0],
            [0, 0,  dt, 1]
        ])
        Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/2/self.mass, 0],
            [0,      dt/self.mass],
            [0, dt*dt/2/self.mass]
        ])
        q1 = self.Sigma_w[0,0]
        q2 = self.Sigma_w[1,1]
        q3 = self.Sigma_w[2,2]
        q4 = self.Sigma_w[3,3]
        Sigma_w_tilde = np.array([
            [dt*q1,         dt*dt/2*q1,                 0,          0],
            [dt*dt/2*q1,    (dt*q2)+(dt*dt*dt/3*q1),    0,          0],
            [0,             0,                          dt*q3,      dt*dt/2*q3],
            [0,             0,                          dt*dt/2*q3, (dt*q4)+(dt*dt*dt/3*q3)],
        ])

        return Atilde, Btilde, Sigma_w_tilde

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    # predict step
    def _predict_Step(self, ctrl_time):
        dt = ctrl_time - self.t
        self.t = ctrl_time

        At, Bt, Sigma = self._discretization_Func(dt)

        self.x = np.matmul(At , self.x) + np.matmul(Bt , self.u)
        self.P = np.matmul(np.matmul(At , self.P) , At.T) + Sigma

    # correction step
    def _correction_Step(self, y):
        self.x = self.x + np.matmul(np.matmul(np.matmul(self.P, self.C.T) , np.linalg.inv(np.matmul(np.matmul(self.C , self.P), self.C.T) + self.Sigma_v)) , (y - np.matmul(self.C , self.x)))
        self.P = self.P - np.matmul(np.matmul(np.matmul(np.matmul(self.P , self.C.T) , np.linalg.inv(np.matmul(np.matmul(self.C , self.P) , self.C.T) + self.Sigma_v)) , self.C) , self.P)

    # when getting the control signal, execution the predict step, update the control signal
    def control_moment(self, u_new, time_now):
        self._predict_Step(time_now)
        self.u = u_new
        # ...

    # when getting the observe info, execution the predict step, and then execution the correction step
    def observe_moment(self, y_new, time_now):
        self._predict_Step(time_now)
        self._correction_Step(y_new)
        # ...

    ## ================================ Edit above here ================================ ^^^^^^^^^^^
    ################################################################################################

# The forward dynamic model. Need NOT edit.
class Model(object):
    # initialization the kalman filter. 
    #   x(k+1) = Ax(k) + Bu(k) + w(k)
    #   y(k) = Cx(k) + v(k)
    #   x(0) ~ N(x_0, P_0)
    def __init__(self, mass, C, Sigma_w, Sigma_v, x_0, P_0):
        self.mass = mass
        self.C = C
    
        self.n = 4
        self.m = 2

        self.Sigma_w = Sigma_w
        self.Sigma_v = Sigma_v

        self.x = np.random.multivariate_normal(x_0.reshape([self.n]), P_0).reshape([self.n, 1])
        self.u = np.zeros([self.m, 1])

        self.t = 0

    def _discretization_Func(self, dt):
        Atilde = np.array([
            [1, 0,  0,  0],
            [dt,1,  0,  0],
            [0, 0,  1,  0],
            [0, 0,  dt, 1]
        ])
        Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/2/self.mass, 0],
            [0,      dt/self.mass],
            [0, dt*dt/2/self.mass]
        ])
        q1 = self.Sigma_w[0,0]
        q2 = self.Sigma_w[1,1]
        q3 = self.Sigma_w[2,2]
        q4 = self.Sigma_w[3,3]
        Sigma_w_tilde = np.array([
            [dt*q1,         dt*dt/2*q1,                 0,          0],
            [dt*dt/2*q1,    (dt*q2)+(dt*dt*dt/3*q1),    0,          0],
            [0,             0,                          dt*q3,      dt*dt/2*q3],
            [0,             0,                          dt*dt/2*q3, (dt*q4)+(dt*dt*dt/3*q3)],
        ])

        return Atilde, Btilde, Sigma_w_tilde

    # system forward dynimic
    def control(self, u, time_now):
        dt = time_now - self.t
        self.t = time_now
        At, Bt, Sigma = self._discretization_Func(dt)
        w = np.random.multivariate_normal(np.zeros([self.n]), Sigma).reshape([self.n, 1])
        self.x = At.dot(self.x) + Bt.dot(self.u) + w
        self.u = u

    # observe the system and return y
    def observe(self, time_now):
        self.control(self.u, time_now)
        v = np.random.multivariate_normal(np.zeros([self.C.shape[0]]), Sigma_v).reshape([self.C.shape[0], 1])
        y = self.C.dot(self.x) + v
        return y

# Random select controlling time and observing time. Then sort by time, generating the time sequence. Need NOT edit.
def generateTimeSequence():
    maxtime = 10
    ctrl_interval = 0.05
    ctrl_sigma = 0.01
    obs_interval = 0.1
    obs_sigma = 0.01
    
    t = 0
    ctrl_time_list = []
    while t < 10:
        dt = np.random.randn(1)*ctrl_sigma + ctrl_interval
        t = t + dt[0]
        if t >= 10:
            break
        ctrl_time_list.append([t, 0])
    t = 0
    obs_time_list = []
    while t < 10:
        dt = np.random.randn(1)*obs_sigma + obs_interval
        t = t + dt[0]
        if t >= 10:
            break
        obs_time_list.append([t, 1])
    
    time_seq = ctrl_time_list + obs_time_list
    time_seq.sort(key=lambda x:x[0])
    return time_seq

# Generate figure object, prepare to plot or save. Need NOT edit.
def generateFig(x_save, p_save, time_seq):
    time_seq = np.stack(time_seq)
    x_save = np.concatenate(x_save, axis=1)
    p_save = np.stack(p_save, axis=0)


    fig_x = plt.figure(0)
    ax = fig_x.subplots(2,2)
    ax[0,0].plot(time_seq[:,0], x_save[0,:].T, label = "true")
    ax[0,0].plot(time_seq[:,0], x_save[4,:].T, label = "est")
    ax[0,0].legend(bbox_to_anchor = (0.0,1.3), loc='upper left')
    ax[0,0].set_title('vx')

    ax[1,0].plot(time_seq[:,0], x_save[1,:].T)
    ax[1,0].plot(time_seq[:,0], x_save[5,:].T)
    ax[1,0].set_title('px')

    ax[0,1].plot(time_seq[:,0], x_save[2,:].T)
    ax[0,1].plot(time_seq[:,0], x_save[6,:].T)
    ax[0,1].set_title('vy')

    ax[1,1].plot(time_seq[:,0], x_save[3,:].T)
    ax[1,1].plot(time_seq[:,0], x_save[7,:].T)
    ax[1,1].set_title('py')

    ctrl_mmt = np.nonzero( 1 - time_seq[:,1])
    obs_mmt  = np.nonzero( time_seq[:,1] )

    ctrl_t = time_seq[ctrl_mmt,:][0,:,:]
    obs_t  = time_seq[obs_mmt, :][0,:,:]

    fig_p = plt.figure(1)
    ax = fig_p.subplots(2,2)
    ax[0,0].plot(time_seq[ctrl_mmt,:][0,:,0], p_save[ctrl_mmt,0,0][0,:], label='predict')
    ax[0,0].plot(time_seq[obs_mmt,:][0,:,0], p_save[obs_mmt,0,0][0,:], label='correct')
    ax[0,0].legend(bbox_to_anchor = (0.0,1.3), loc='upper left')
    ax[0,0].set_title('P[0,0]')
    ax[0,1].plot(time_seq[ctrl_mmt,:][0,:,0], p_save[ctrl_mmt,1,1][0,:], label='predict')
    ax[0,1].plot(time_seq[obs_mmt,:][0,:,0], p_save[obs_mmt,1,1][0,:], label='correct')
    ax[0,1].set_title('P[1,1]')
    ax[1,0].plot(time_seq[ctrl_mmt,:][0,:,0], p_save[ctrl_mmt,2,2][0,:], label='predict')
    ax[1,0].plot(time_seq[obs_mmt,:][0,:,0], p_save[obs_mmt,2,2][0,:], label='correct')
    ax[1,0].set_title('P[2,2]')
    ax[1,1].plot(time_seq[ctrl_mmt,:][0,:,0], p_save[ctrl_mmt,3,3][0,:], label='predict')
    ax[1,1].plot(time_seq[obs_mmt,:][0,:,0], p_save[obs_mmt,3,3][0,:], label='correct')
    ax[1,1].set_title('P[3,3]')

    return fig_x, fig_p



if __name__ == "__main__":
    # set random seed for repeatable result.
    np.random.seed(0)

    # set model param
    mass = 1
    C = np.array([
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    Sigma_w = np.diag([1,1,1,1])/1000
    Sigma_v = np.eye(2)/5000
    x_0 = np.zeros([4,1])
    P_0 = np.eye(4)/500

    # initialize (Kalman filter) and (forward dynamic model)
    kf = KalmanFilter(mass, C, Sigma_w, Sigma_v, x_0, P_0)
    mdl = Model(mass, C, Sigma_w, Sigma_v, x_0, P_0)

    # control signal generater
    u = lambda t:np.array([[np.sin(1*t)], [np.cos(1*t)]])

    # set controlling/observing time sequence
    time_seq = generateTimeSequence()

    # prepare to save data
    x_save = []
    p_save = []
    
    # start simulation
    t = 0
    for iter in time_seq:
        t = iter[0]
        if iter[1] < 0.5:
            # controlling moment
            mdl.control(u(t), t)
            kf.control_moment(u(t), t)
        else:
            # observing momont
            y = mdl.observe(t)
            kf.observe_moment(y, t)
        # record true/esti data
        x_save.append( np.concatenate([mdl.x, kf.x], axis=0) )
        p_save.append(kf.P)
    
    # visualization
    fig_x, fig_p = generateFig(x_save, p_save, time_seq)
    fig_x.savefig('fig_x.png')
    fig_p.savefig('fig_p.png')

    pass




        

