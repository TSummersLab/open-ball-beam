import autograd.numpy as np
import autograd.numpy.linalg as la
import autograd.numpy.random as npr
from scipy.linalg import solve_discrete_lyapunov, solve_discrete_are, expm

import matplotlib.pyplot as plt

from pickle_io import pickle_export
from settings import DT


def dlyap(A, Q):
    return solve_discrete_lyapunov(a=np.copy(A).T, q=np.copy(Q))


def dare(A, B, Q, R):
    return solve_discrete_are(a=np.copy(A), b=np.copy(B), q=np.copy(Q), r=np.copy(R))


def get_gain(A, B, Q, R):
    P = dare(A, B, Q, R)
    K = -la.solve(R + np.dot(B.T, np.dot(P, B)), np.dot(B.T, np.dot(P, A)))
    return K


# Model data
damp = 1e-3  # 1/second
gravity = 9.81  # meters/second/second

Ac = np.array([[0, 1],
               [0, -damp]])
Bc = np.array([[0], [-gravity]])

n2, m2, p2 = 2, 1, 1
A2 = expm(DT*Ac)  # ~= np.eye(n2) + DT*Ac
B2 = DT*Bc
C2 = np.array([[1, 0]])

# Add integral control of position via augmented state & output
n3, m3, p3 = 3, 1, 1
A3 = np.block([[A2, np.zeros([n2, 1])],
              [np.array([1, 0]), np.array([1])]])
B3 = np.block([[B2], [0]])

# Add penalty on control differences via augmented state
n, m, p = 4, 1, 1
A4 = np.block([[A3, B3],
               [np.zeros([m3, n3]), np.eye(m3)]])
B4 = np.block([[B3], [np.eye(m3)]])

# State & control penalty weights
Q4 = np.diag([10.0, 1.0, 0.001, 1.0])  # position, velocity, integral of position, control effort
R4 = np.diag([1.0])  # control difference

# Process & measurement noise covariances
W2 = np.diag([1e-6, 1e-5])
V2 = np.diag([1e-4])


# Control & estimator design
K = get_gain(A4, B4, Q4, R4)
L2 = get_gain(A2.T, C2.T, W2, V2).T

# Export controller parameters
controller_data = dict(A=A2, B=B2[:, 0], C=C2[0], K=K[0], L=L2[:, 0])
pickle_export(dirname_out='.', filename_out='controller_data.pickle', data=controller_data)

# def print_arduino_vector(vector, var_name=None):
#     if var_name is not None:
#         print('%s = ' % var_name, end='')
#     print('{' + ', '.join(['%.6f' % num for num in vector]) + '}')
#
# print_arduino_vector(controller_data.K, var_name='K')
# print_arduino_vector(controller_data.L, var_name='L')


# Simulation setup
total_time = 10  # in seconds
T = int(total_time/DT)
z_hist = np.zeros([T+1, n])
z_est_hist = np.zeros([T+1, n])

u_hist = np.zeros([T, m])
y_hist = np.zeros([T, p])
u0 = 0
z0 = np.array([0.1, 0, 0.1, u0])
x0 = z0[0:2]
z_hist[0] = z0
z_est_hist[0] = z0
x_est_hist = z_est_hist[:, 0:n2]
y_hist[0] = z0[0]

u = np.copy(u0)
z = np.copy(z0)
z_est = np.copy(z0)
x = np.copy(z[0:2])
x_est = np.copy(z_est[0:2])

AL2 = A2 + np.dot(L2, C2)

# Simulation
from controller import LQGController
from reference import ConstantReference
from simulator import Simulator
system = Simulator(x0=x0)
K = K[0]
B2 = B2[:, 0]
L2 = L2[:, 0]

# TODO get simulation and history data from interface.py and just plot it here
#
#
#
# # Plot step response simulation results
# plt.close('all')
# fig, ax = plt.subplots(nrows=4, sharex=True, figsize=(8, 10))
# ax[0].plot(z_hist[:, 0], c='C0', label='true')
# ax[0].plot(y_hist[:, 0], c='C1', alpha=0.7, label='measured')
# ax[0].plot(z_est_hist[:, 0], c='C2', alpha=0.5, label='estimated')
# ax[0].legend()
#
# ax[1].plot(z_hist[:, 1], c='C0', label='true')
# ax[1].plot(z_est_hist[:, 1], c='C2', alpha=0.5, label='estimated')
# ax[1].legend()
#
# ax[2].plot(z_hist[:, 2], c='C0', label='true')
# ax[2].plot(z_est_hist[:, 2], c='C2', alpha=0.5, label='estimated')
# ax[2].legend()
#
# ax[-1].plot(u_hist)
#
# ax[0].set_ylabel('position')
# ax[1].set_ylabel('velocity')
# ax[2].set_ylabel('position integral')
#
# ax[-1].set_ylabel('input')
# ax[-1].set_xlabel('time')
#
# fig.tight_layout()
# plt.show()
