"""
Generic program for fitting cubic spline for different types of constraints
Book reference: J. Craig - Robot Mechanics and Control

Written by
Suraj Borate at Robiotar Labs

"""

import numpy as np

import matplotlib.pyplot as plt


def get_constraint(n,t,b):
    """mention type of constraint
    n=1 for position constraint
    n=2 for velocity
    n=3 for acceleration
    """
    if n ==1:
        C = [1,1*t,1*t*t,1*t*t*t]

    if n == 2:
        C = [0,1,2*t,3*t*t]

    if n == 3:
        C = [0,0,2,6*t]

    return C,b

def fit_cubic(C1,C2,C3,C4,t0,tf):
    t = np.linspace(t0,tf,100)
    A = np.array([C1[0],C2[0],C3[0],C4[0]])
    b = np.array([C1[1],C2[1],C3[1],C4[1]])
    a = np.linalg.solve(A,b)
    print(a)
    theta = a[0] + a[1] * t + a[2] * t * t + a[3] * t * t * t
    cubic_eq = [a[0],a[1],a[2],a[3]]
    print(cubic_eq)
    theta_dot = a[1] + 2 * a[2] * t + 3 * a[3] * t * t
    theta_dot_dot = 2 * a[2] + 6 * a[3] * t
    return theta,theta_dot,theta_dot_dot

def plot_cubic(theta,theta_dot,theta_dot_dot,t0,tf,num =100):
    t = np.linspace(t0,tf,num)
    plt.figure()
    plt.plot(t,theta)
    plt.figure()
    plt.plot(t,theta_dot)
    plt.figure()
    plt.plot(t,theta_dot_dot)
    plt.show()

C1 = get_constraint(1,0,2)
C2 = get_constraint(1,10,8)
C3 = get_constraint(2,0,0)
C4 = get_constraint(2,10,0)

theta, theta_dot, theta_dot_dot = fit_cubic(C1,C2,C3,C4,0,10)
plot_cubic(theta,theta_dot,theta_dot_dot,0,10,100)
