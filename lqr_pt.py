#! /usr/bin/python

import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

def dlqr(A,B,Q,R):
    """
    Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # first, solve the ricatti equation
    P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*P*B+R)*(B.T*P*A))
    return -K

l = 1.64 # rod length is 2l
m = 550.0 # rod 6 mm diameter, 44cm length, 7856 kg/m^3
M = .4
dt = .02 # 20 ms
g = 9.8
Iz = 900.0
lf = 1.13
lr = 0.51
cf = 150000.0
cr = 150000.0
vx = 3.0

a22 = -(cf + cr) / (m * vx)
a23 = (cf + cr) / m
a24 = (lr * cr - lf * cf) / (m * vx)
a42 = (lr * cr - lf * cf) / (Iz * vx)
a43 = (lf * cf - lr * cr) / (Iz)
a44 = -(lf**2 * cf + lr**2 * cr) / (Iz * vx)
b21 = cf / m
b41 = lf * cf / Iz

A = np.matrix([[0,   1,   0,   0],
               [0, a22, a23, a24],
               [0,   0,   0,   1],
               [0, a42, a43, a44]])
B = np.matrix([[0],
               [b21],
               [0],
               [b41]])


print "A: ", A
print "B: ", B

Q = np.matrix("1 0 0 0; 0 0 0 0 ; 0 0 0 0; 0 0 0 0")
R = np.matrix("10000")


K = dlqr(A,B,Q,R)
print "K: ", K
print "double c[] = {%f, %f, %f, %f};" % (K[0,0], K[0,1], K[0,2], K[0,3])

nsteps = 250
time = np.linspace(0, 2, nsteps, endpoint=True)
xk = np.matrix("1 ; 0 ; 0.1 ; 0")

X = []
T = []
U = []

for t in time:
    uk = K * xk
    print uk
    X.append(xk[0,0])
    T.append(xk[2,0])
    #v = xk[1,0]
    #force = uk[0,0]
    #accel = force/(M+m)
    #u = ((1-.404)*v + dt*accel)/.055/10
    u = uk[0, 0]
    U.append(u)
    xk = A * xk + B * uk

plt.plot(time, X, label="Position error, meters")
#plt.plot(time, T, label='YAW error, radians')
plt.plot(time, U, label='Steering angle error, radians')

plt.legend(loc='upper right')
#plt.grid()
plt.show()
