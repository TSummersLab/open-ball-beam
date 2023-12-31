import os

import numpy as np
import osqp
from scipy import sparse

from ballbeam.common.pickle_io import pickle_export, pickle_import
from ballbeam.static import CONFIGURATION_PATH

# Pathing
DIRECTORY_NAME_LQG = CONFIGURATION_PATH.joinpath("controller", "lqg")
DIRECTORY_NAME_MPC = CONFIGURATION_PATH.joinpath("controller", "mpc")


def make_design_data():
    # Copy from LQG
    path_in = os.path.join(DIRECTORY_NAME_LQG, "design_data.pickle")
    design_data = pickle_import(path_in)

    # Terminal state penalty
    design_data["controller"]["QN4"] = 10 * design_data["controller"]["Q4"]

    # Constraints
    xmax = np.array(
        [1.0, 10.0, 100.0, 0.08],
    )  # position (m), velocity (m/s), integral of position (m), control effort (rad)
    xmin = -xmax
    umax = np.array([0.1])  # control difference (rad)
    umin = -umax

    constraint_design_data = {"xmin": xmin, "xmax": xmax, "umin": umin, "umax": umax}
    for key, val in constraint_design_data.items():
        design_data["controller"][key] = val

    # Prediction horizon
    N = 20
    design_data["N"] = N

    pickle_export(dirname_out=DIRECTORY_NAME_MPC, filename_out="design_data.pickle", data=design_data)
    return design_data


def make_controller_params():
    # Copy from LQG
    path_in = os.path.join(DIRECTORY_NAME_LQG, "controller_params.pickle")
    controller_params = pickle_import(path_in)
    pickle_export(dirname_out=DIRECTORY_NAME_MPC, filename_out="controller_params.pickle", data=controller_params)
    return controller_params


def make_problem(A, B, Q, QN, R, xmin, xmax, umin, umax, N):
    nx, nu = B.shape
    xr = np.zeros(nx)  # Reference state

    # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    # Quadratic objective
    P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN, sparse.kron(sparse.eye(N), R)], format="csc")
    # Linear objective
    q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr), np.zeros(N * nu)])
    # Linear dynamics
    Ax = sparse.kron(sparse.eye(N + 1), -sparse.eye(nx)) + sparse.kron(sparse.eye(N + 1, k=-1), A)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), B)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-np.zeros(nx), np.zeros(N * nx)])
    ueq = leq
    # Input and state constraints
    Aineq = sparse.eye((N + 1) * nx + N * nu)
    lineq = np.hstack([np.kron(np.ones(N + 1), xmin), np.kron(np.ones(N), umin)])
    uineq = np.hstack([np.kron(np.ones(N + 1), xmax), np.kron(np.ones(N), umax)])
    # OSQP constraints
    Acon = sparse.vstack([Aeq, Aineq], format="csc")
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, Acon, l, u, warm_start=True, verbose=False)
    return l, u, prob


def make_codegen_solver() -> None:
    design_data_path = DIRECTORY_NAME_MPC.joinpath("design_data.pickle")
    controller_design_data = pickle_import(design_data_path)

    A, B, Q, QN, R = (controller_design_data["controller"][key] for key in ["A4", "B4", "Q4", "QN4", "R4"])
    xmin, xmax, umin, umax = (controller_design_data["controller"][key] for key in ["xmin", "xmax", "umin", "umax"])
    N = controller_design_data["N"]

    nx, nu = B.shape
    l_var, u_var, prob = make_problem(A, B, Q, QN, R, xmin, xmax, umin, umax, N)

    codegen_dir = DIRECTORY_NAME_MPC.joinpath("codegen")
    prob.codegen(codegen_dir, python_ext_name="emosqp", force_rewrite=True)


def main() -> None:
    make_design_data()
    make_controller_params()
    make_codegen_solver()


if __name__ == "__main__":
    main()
