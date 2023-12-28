import numpy as np
import numpy.linalg as la
from scipy.linalg import solve_discrete_lyapunov, solve_discrete_are, expm

from ballbeam.common.utility import print_arduino_vector
from ballbeam.common.pickle_io import pickle_export, pickle_import
from ballbeam.configurators.configs import CONFIG
from ballbeam.static import CONFIGURATION_PATH


# Pathing
dirname_out = CONFIGURATION_PATH.joinpath('controller', 'lqg')


def dlyap(A, Q):
    return solve_discrete_lyapunov(a=np.copy(A).T, q=np.copy(Q))


def dare(A, B, Q, R):
    return solve_discrete_are(a=np.copy(A), b=np.copy(B), q=np.copy(Q), r=np.copy(R))


def dare_gain(A, B, Q, R):
    P = dare(A, B, Q, R)
    K = -la.solve(R + np.dot(B.T, np.dot(P, B)), np.dot(B.T, np.dot(P, A)))
    return K


def make_design_data():
    # Continuous-time model
    Ac = np.array([[0, 1],
                   [0, -CONFIG.model.DAMP_SCALED]])
    Bc = np.array([[0],
                   [-CONFIG.model.GRAVITY_SCALED]])
    Cc = np.array([[1, 0]])

    # Convert continuous-time model to discrete-time
    n2, m2, p2 = 2, 1, 1
    A2 = expm(CONFIG.hardware.COMM.DT*Ac)  # ~= np.eye(n2) + DT*Ac
    B2 = CONFIG.hardware.COMM.DT*Bc
    C2 = np.copy(Cc)

    # Add integral control of position via augmented state
    n3, m3, p3 = 3, 1, 1
    A3 = np.block([[A2, np.zeros([n2, 1])],
                   [np.array([CONFIG.hardware.COMM.DT, 0]), np.array([1])]])
    B3 = np.block([[B2],
                   [0]])

    # Add penalty on control differences via augmented state
    n, m, p = 4, 1, 1
    A4 = np.block([[A3, B3],
                   [np.zeros([m3, n3]), np.eye(m3)]])
    B4 = np.block([[B3],
                   [np.eye(m3)]])

    # State & control penalty weights
    Q4 = np.diag([80.0, 1.0, 200.0, 0.1])  # position, velocity, integral of position, control effort
    R4 = np.diag([4.0])  # control difference

    # Process & measurement noise covariances
    W2 = np.diag([1e-6, 1e-6])  # position state, velocity state
    V2 = np.diag([1e-5])  # position measurement

    # Export controller design parameters
    controller_design_data = dict(controller=dict(A4=A4, B2=B2, B4=B4, Q4=Q4, R4=R4),
                                  estimator=dict(A2=A2, C2=C2, W2=W2, V2=V2))

    pickle_export(dirname_out=dirname_out, filename_out='design_data.pickle', data=controller_design_data)
    return controller_design_data


def make_controller_params(show_print_arduino=False):
    controller_design_data = pickle_import(dirname_out.joinpath('design_data.pickle'))

    A4, B2, B4, Q4, R4 = [controller_design_data['controller'][key] for key in ['A4', 'B2', 'B4', 'Q4', 'R4']]
    A2, C2, W2, V2 = [controller_design_data['estimator'][key] for key in ['A2', 'C2', 'W2', 'V2']]

    # Control & estimator design
    K4 = dare_gain(A4, B4, Q4, R4)
    L2 = dare_gain(A2.T, C2.T, W2, V2)
    L2 = -L2.T

    # Export controller parameters
    controller_params = dict(A=A2, B=B2[:, 0], C=C2[0], K=K4[0], L=L2[:, 0])
    pickle_export(dirname_out=dirname_out, filename_out='controller_params.pickle', data=controller_params)

    if show_print_arduino:
        print_arduino_vector(controller_params['K'], var_name='K')
        print_arduino_vector(controller_params['L'], var_name='L')

    return controller_params


def main(show=False):
    make_design_data()
    make_controller_params(show_print_arduino=show)
    return


if __name__ == '__main__':
    main(show=True)


# # Simulation setup
# total_time = 10  # in seconds
# T = int(total_time/DT)
# z_hist = np.zeros([T+1, n])
# z_est_hist = np.zeros([T+1, n])
#
# u_hist = np.zeros([T, m])
# y_hist = np.zeros([T, p])
# u0 = 0
# z0 = np.array([0.1, 0, 0.1, u0])
# x0 = z0[0:2]
# z_hist[0] = z0
# z_est_hist[0] = z0
# x_est_hist = z_est_hist[:, 0:n2]
# y_hist[0] = z0[0]
#
# u = np.copy(u0)
# z = np.copy(z0)
# z_est = np.copy(z0)
# x = np.copy(z[0:2])
# x_est = np.copy(z_est[0:2])
#
# AL2 = A2 + np.dot(L2, C2)

# # Simulation
# from controller import LQGController
# from reference import ConstantReference
# from simulator import Simulator
# system = Simulator(x0=x0)
# K = K[0]
# B2 = B2[:, 0]
# L2 = L2[:, 0]

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
