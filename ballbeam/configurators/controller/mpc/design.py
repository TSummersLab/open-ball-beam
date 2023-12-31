"""Design of MPC controllers."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import osqp  # type: ignore[import]
from scipy import sparse  # type: ignore[import]

from ballbeam.common.pickle_io import pickle_export, pickle_import
from ballbeam.static import CONFIGURATION_PATH

if TYPE_CHECKING:
    from ballbeam.common.types import NDA


DIRECTORY_NAME_LQG = CONFIGURATION_PATH.joinpath("controller", "lqg")
DIRECTORY_NAME_MPC = CONFIGURATION_PATH.joinpath("controller", "mpc")


def make_design_data() -> dict[str, NDA | int]:
    """Make design data."""
    # Copy from LQG
    path_in = DIRECTORY_NAME_LQG.joinpath("design_data.pickle")
    design_data = pickle_import(path_in)

    # Terminal state penalty
    design_data["controller"]["QN4"] = 10 * design_data["controller"]["Q4"]

    # Constraints
    # position (m), velocity (m/s), integral of position (m), control effort (rad)
    xmax = np.array([1.0, 10.0, 100.0, 0.08])
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


def make_controller_params() -> dict[str, NDA]:
    """Make controller parameters."""
    # Copy from LQG
    path_in = DIRECTORY_NAME_LQG.joinpath("controller_params.pickle")
    controller_params = pickle_import(path_in)
    pickle_export(dirname_out=DIRECTORY_NAME_MPC, filename_out="controller_params.pickle", data=controller_params)
    return controller_params


def make_problem(
    A: NDA,
    B: NDA,
    Q: NDA,
    QN: NDA,
    R: NDA,
    xmin: NDA,
    xmax: NDA,
    umin: NDA,
    umax: NDA,
    N: int,
) -> osqp.OSQP:
    """Make an OSQP optimal control problem."""
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
    lower_constraint_var = np.hstack([leq, lineq])
    upper_constraint_var = np.hstack([ueq, uineq])

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, Acon, lower_constraint_var, upper_constraint_var, warm_start=True, verbose=False)
    return prob


def make_codegen_solver() -> None:
    """Make a code-generated solver."""
    design_data_path = DIRECTORY_NAME_MPC.joinpath("design_data.pickle")
    controller_design_data = pickle_import(design_data_path)

    A, B, Q, QN, R = (controller_design_data["controller"][key] for key in ["A4", "B4", "Q4", "QN4", "R4"])
    xmin, xmax, umin, umax = (controller_design_data["controller"][key] for key in ["xmin", "xmax", "umin", "umax"])
    N = controller_design_data["N"]

    prob = make_problem(A, B, Q, QN, R, xmin, xmax, umin, umax, N)

    codegen_dir = DIRECTORY_NAME_MPC.joinpath("codegen")
    prob.codegen(codegen_dir, python_ext_name="emosqp", force_rewrite=True)


def main() -> None:
    """Run the main function."""
    make_design_data()
    make_controller_params()
    make_codegen_solver()


if __name__ == "__main__":
    main()
