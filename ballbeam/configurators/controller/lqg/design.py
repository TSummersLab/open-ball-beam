"""Design of LQG controllers."""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import numpy.linalg as la
from scipy.linalg import expm, solve_discrete_are, solve_discrete_lyapunov  # type: ignore[import]

from ballbeam.common.pickle_io import pickle_export, pickle_import
from ballbeam.common.utility import print_arduino_vector
from ballbeam.configurators.configs import CONFIG
from ballbeam.static import CONFIGURATION_PATH

if TYPE_CHECKING:
    from ballbeam.common.types import NDA

DIRECTORY_NAME_OUT = CONFIGURATION_PATH.joinpath("controller", "lqg")


def dlyap(A: NDA, Q: NDA) -> NDA:
    """Solve a discrete-time Lyapunov equation.

    Thin wrapper around scipy.linalg.solve_discrete_lyapunov.
    """
    return solve_discrete_lyapunov(a=np.copy(A).T, q=np.copy(Q))


def dare(A: NDA, B: NDA, Q: NDA, R: NDA, *, return_gain: bool = False) -> NDA | tuple[NDA, NDA]:
    """Solve a discrete-time algebraic Riccati equation.

    Thin wrapper around scipy.linalg.solve_discrete_are.
    """
    P = solve_discrete_are(a=np.copy(A), b=np.copy(B), q=np.copy(Q), r=np.copy(R))
    K = -la.solve(R + np.dot(B.T, np.dot(P, B)), np.dot(B.T, np.dot(P, A)))
    return P, K if return_gain else P


def make_design_data() -> dict[str, dict[str, NDA]]:
    """Make design data."""
    # Continuous-time model
    Ac = np.array([[0, 1], [0, -CONFIG.model.DAMP_SCALED]])
    Bc = np.array([[0], [-CONFIG.model.GRAVITY_SCALED]])
    Cc = np.array([[1, 0]])

    # Convert continuous-time model to discrete-time
    n2, m2, p2 = 2, 1, 1  # noqa: F841
    A2 = expm(CONFIG.hardware.COMM.DT * Ac)  # ~= np.eye(n2) + DT*Ac
    B2 = CONFIG.hardware.COMM.DT * Bc
    C2 = np.copy(Cc)

    # Add integral control of position via augmented state
    n3, m3, p3 = 3, 1, 1  # noqa: F841
    A3 = np.block([[A2, np.zeros([n2, 1])], [np.array([CONFIG.hardware.COMM.DT, 0]), np.array([1])]])
    B3 = np.block([[B2], [0]])

    # Add penalty on control differences via augmented state
    n, m, p = 4, 1, 1  # noqa: F841
    A4 = np.block([[A3, B3], [np.zeros([m3, n3]), np.eye(m3)]])
    B4 = np.block([[B3], [np.eye(m3)]])

    # State & control penalty weights
    Q4 = np.diag([80.0, 1.0, 200.0, 0.1])  # position, velocity, integral of position, control effort
    R4 = np.diag([4.0])  # control difference

    # Process & measurement noise covariances
    W2 = np.diag([1e-6, 1e-6])  # position state, velocity state
    V2 = np.diag([1e-5])  # position measurement

    # Export controller design parameters
    controller_design_data = {
        "controller": {"A4": A4, "B2": B2, "B4": B4, "Q4": Q4, "R4": R4},
        "estimator": {"A2": A2, "C2": C2, "W2": W2, "V2": V2},
    }

    pickle_export(dirname_out=DIRECTORY_NAME_OUT, filename_out="design_data.pickle", data=controller_design_data)
    return controller_design_data


def make_controller_params(*, show_print_arduino: bool = False) -> dict[str, NDA]:
    """Make controller parameters."""
    controller_design_data = pickle_import(DIRECTORY_NAME_OUT.joinpath("design_data.pickle"))

    A4, B2, B4, Q4, R4 = (controller_design_data["controller"][key] for key in ["A4", "B2", "B4", "Q4", "R4"])
    A2, C2, W2, V2 = (controller_design_data["estimator"][key] for key in ["A2", "C2", "W2", "V2"])

    # Control & estimator design
    _, K4 = dare(A4, B4, Q4, R4, return_gain=True)
    _, L2 = dare(A2.T, C2.T, W2, V2, return_gain=True)
    L2 = -L2.T

    # Export controller parameters
    controller_params = {"A": A2, "B": B2[:, 0], "C": C2[0], "K": K4[0], "L": L2[:, 0]}
    pickle_export(dirname_out=DIRECTORY_NAME_OUT, filename_out="controller_params.pickle", data=controller_params)

    if show_print_arduino:
        print_arduino_vector(controller_params["K"], var_name="K")
        print_arduino_vector(controller_params["L"], var_name="L")

    return controller_params


def main(*, show: bool = False) -> None:
    """Run the main function."""
    make_design_data()
    make_controller_params(show_print_arduino=show)


if __name__ == "__main__":
    main(show=True)
