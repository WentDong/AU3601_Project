import numpy as np
from matplotlib import pyplot as plt


'''
@param X_0 initial states [vx_0, px_0, vy_0, py_0]
@param F   force 
@return X  states
'''
def forward_dynamics(X_0, F, dt=0.01, m = 1):
    Len = F[:, 0].size # number of time steps 
    Dim = X_0.size # state-space model dimension

    # initialize the trajectory, each column corresponds to a time step
    X = np.zeros(shape=(Len, Dim)) 
    X[0, :] = X_0 # put the initial value into the first column

    # Matrix A
    A = np.array([[1, 0, 0, 0],
                  [dt, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, dt, 1]])
    # Matrix B
    B = np.array([[dt/m, 0],
                  [dt**2 / 2 / m, 0],
                  [0, dt/m],
                  [0, dt**2/2/m]])

    # print(A.shape, B.shape, X.shape, X[0,:].shape, F.shape)

    # iterate over time
    for i in range(1, Len):
        # print(i)
        # Dynamic model
        X[i, :] = A.dot(X[i - 1, :])
        X[i, :] += B.dot(F[i -1, :])
        # Add noise
        w = np.random.multivariate_normal( \
            np.zeros(Dim), Sigma)
        X[i, :] += w
    return X

if __name__ == '__main__':
    # partical weight (kg)
    m = 1 
    # Noise variation q3==Q1 q4==Q1
    q1 = 0.00004
    q3 = 0.00004
    q2 = 0.003
    q4 = 0.003
    # Sampling period
    dt = 0.01

    # covariance for the process noise
    Sigma = np.array(  [[dt * q1,           dt * dt /2 * q1  ,   0,  0],
                        [dt * dt /2 * q1,   dt * q2 + dt ** 3,   0,  0],
                        [0, 0,  dt * q3, dt**2 / 2 * q3],
                        [0, 0, dt **2/2 * q3, dt * q4 + dt**3 /3 * q3]])

    # total simulation time
    time = 30

    # Sampling time sequences
    t = np.arange(0, int(time/dt)) * dt

    # force input
    F_x = -np.sin(t)
    F_y = np.cos(t)
    F = np.stack([F_x, F_y], axis=1)

    # init state X_0 = ([vx_0, px_0, vy_0, py_0])
    X_0 = np.array([1, 0, 0, 0])
    X = forward_dynamics(X_0, F)

    # draw picture
    fig, ax = plt.subplots(2, 2) # 2*2 plots
    ax[0, 0].plot(t, F_x)
    ax[0, 0].set_title("t-Fx")
    ax[1, 0].plot(t, X[:, 1])
    ax[1, 0].set_title("t-px")
    ax[0, 1].plot(t, F_y)
    ax[0, 1].set_title("t-Fy")
    ax[1, 1].plot(t, X[:, 3])
    ax[1, 1].set_title("t-py")

    plt.figure(2)
    plt.plot(X[:, 1], X[:, 3])
    plt.title("Moving Trajactory")
    plt.show()


