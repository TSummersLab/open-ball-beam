"""Classes to represent controllers of the Open Ball & Beam."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from ballbeam.common.extramath import mix
from ballbeam.common.pickle_io import pickle_import
from ballbeam.configurators.configs import CONFIG
from ballbeam.static import CONFIGURATION_PATH


class Controller:
    """Base class for a controller.

    A controller is responsible for taking observations, and possibly other auxiliary signals,
    and producing a control action as output.
    """

    def __init__(self, error_mix: float = 1.0, error_diff_mix: float = 1.0, *, use_anti_windup: bool = False) -> None:
        """Initialize."""
        # Settings
        self.error_mix = error_mix  # 1.0 is no smoothing, << 1.0 is lots of smoothing
        self.error_diff_mix = error_diff_mix  # 1.0 is no smoothing, << 1.0 is lots of smoothing
        self.use_anti_windup = use_anti_windup

        # Initialize self state
        self._u = 0.0  # control action
        self._z = np.zeros(4)  # augmented state estimate

        self.error = 0.0
        self.error_sum = 0.0
        self.error_last = 0.0
        self.error_diff = 0.0

        self.saturated = False
        self.ball_removed = False

    def update_aux(self, *, saturated: bool = False, ball_removed: bool = False) -> None:
        """Update auxiliary internal states.

        Args:
        ----
        saturated: true if actuator is currently saturated
        ball_removed: true if ball is currently removed from beam
        """
        self.saturated = saturated
        self.ball_removed = ball_removed

    def error_update(self, observation: float, setpoint: float) -> None:
        """Update error based on current observation and setpoint."""
        error_new = observation - setpoint
        self.error = mix(error_new, self.error, self.error_mix)

        error_diff_new = (error_new - self.error_last) / CONFIG.hardware.COMM.DT
        self.error_diff = mix(error_diff_new, self.error_diff, self.error_diff_mix)

        # This logic provides anti-windup functionality
        if self.use_anti_windup and self.saturated:
            pass
        else:
            self.error_sum += self.error * CONFIG.hardware.COMM.DT

        # Store the last error
        self.error_last = self.error

    def my_update(self, t: int) -> None:
        """Update additional internal state."""

    def update(self, observation: float, setpoint: float, t: int) -> None:
        """Update internal state of the controller.

        Args:
        ----
        observation: observed value, in meters
        setpoint: target observation value, in meters
        t: time index since start
        """
        self.error_update(observation, setpoint)

        # This logic provides deactivation upon ball removal.
        if self.ball_removed:
            self.error = 0
            self.error_sum = 0
        else:
            self.my_update(t)

    @property
    def action(self) -> float:
        """Controller's current action."""
        if self.ball_removed:
            self._u = 0.0
        return self._u

    @property
    def state_estimate(self) -> npt.NDArray[np.float64]:
        """Controller's current state estimate."""
        if self.ball_removed:
            self._z = np.zeros(4)
        return self._z


class SineController(Controller):
    """Sine controller that just generates a sine wave.

    This is not a closed-loop feedback controller.
    """

    def __init__(self, freq: float = 0.3, beam_angle_max_deg: float = 2.0) -> None:
        """Initialize."""
        super().__init__()
        self.freq = freq  # in Hz
        self.beam_angle_max_deg = beam_angle_max_deg  # in degrees

    def my_update(self, t: int) -> None:
        """Update internal action."""
        beam_angle_max_rad = self.beam_angle_max_deg / CONFIG.constants.RAD2DEG
        self._u = beam_angle_max_rad * np.sin(2 * np.pi * self.freq * t * CONFIG.hardware.COMM.DT)


class PIDController(Controller):
    """Proportional-integral-derivative (PID) controller with exponential smoothing filters."""

    def __init__(self, controller_params_path: str | None = None) -> None:
        """Initialize."""
        super().__init__()
        if controller_params_path is None:
            controller_params_path = CONFIGURATION_PATH.joinpath("controller", "pid", "controller_params.pickle")
        controller_params = pickle_import(controller_params_path)

        self.kp = controller_params["kp"]
        self.ki = controller_params["ki"]
        self.kd = controller_params["kd"]

        self.error_mix = controller_params["error_mix"]
        self.error_diff_mix = controller_params["error_diff_mix"]

        self.use_anti_windup = controller_params["use_anti_windup"]

    def my_update(self, t: int) -> None:  # noqa: ARG002
        """Compute internal control action and update internal state estimate."""
        # Action
        u_p = self.kp * self.error
        u_i = self.ki * self.error_sum
        u_d = self.kd * self.error_diff
        self._u = u_p + u_i + u_d

        # State estimate
        self._z = np.array([self.error, self.error_diff, self.error_sum, self._u])


class LQGController(Controller):
    """Linear quadratic Gaussian (LQG) with integral feedback & control difference penalization."""

    def __init__(self, controller_params_path: str | None = None) -> None:
        """Initialize."""
        super().__init__()
        if controller_params_path is None:
            controller_params_path = CONFIGURATION_PATH.joinpath("controller", "lqg", "controller_params.pickle")
        controller_params = pickle_import(controller_params_path)

        self.A = controller_params["A"]
        self.B = controller_params["B"]
        self.C = controller_params["C"]
        self.K = controller_params["K"]
        self.L = controller_params["L"]

        self.AL = self.A - np.dot(self.L[:, None], self.C[None, :])

    def my_update(self, t: int) -> None:  # noqa: ARG002
        """Update internal state estimate and control action."""
        # State estimate using LQE
        # x is the physical state estimate
        # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
        # LQE is only needed for x since error_sum and action are known exactly
        x = self._z[0:2]
        x = np.dot(self.AL, x) + np.dot(self.B, self._u) + np.dot(self.L, self.error)
        self._z = np.hstack([x, self.error_sum, self._u])

        # Control action using LQR
        du = np.dot(self.K, self._z)
        self._u += du


class MPCController(Controller):
    """Model predictive control (MPC) with integral feedback & control difference penalization.

    Same as LQG but with state and input constraint handling.
    """

    def __init__(self, controller_params_path: str | None = None) -> None:
        """Initialize."""
        super().__init__()

        from ballbeam.common import emosqp

        self.solver = emosqp

        if controller_params_path is None:
            controller_params_path = CONFIGURATION_PATH.joinpath("controller", "mpc", "controller_params.pickle")
        controller_params = pickle_import(controller_params_path)

        self.A = controller_params["A"]
        self.B = controller_params["B"]
        self.C = controller_params["C"]
        self.K = controller_params["K"]
        self.L = controller_params["L"]

        self.AL = self.A - np.dot(self.L[:, None], self.C[None, :])

        design_data_path = CONFIGURATION_PATH.joinpath("controller", "mpc", "design_data.pickle")
        controller_design_data = pickle_import(design_data_path)

        A4, B4, Q4, QN4, R4 = (controller_design_data["controller"][key] for key in ["A4", "B4", "Q4", "QN4", "R4"])
        xmin, xmax, umin, umax = (controller_design_data["controller"][key] for key in ["xmin", "xmax", "umin", "umax"])
        self.N = controller_design_data["N"]
        self.nx, self.nu = B4.shape
        self.bound_lwr, self.bound_upr = self.make_bounds(xmin, xmax, umin, umax)

    def make_bounds(
        self,
        xmin: float,
        xmax: float,
        umin: float,
        umax: float,
    ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """Make the bound constraints for the optimal control problem."""
        # Input and state constraints
        # Equality constraints
        lower_equality_bounds = np.hstack([np.zeros(self.nx), np.zeros(self.N * self.nx)])
        upper_equality_bounds = lower_equality_bounds

        # Inequality constraints
        lower_inequality_bounds = np.hstack([np.kron(np.ones(self.N + 1), xmin), np.kron(np.ones(self.N), umin)])
        upper_inequality_bounds = np.hstack([np.kron(np.ones(self.N + 1), xmax), np.kron(np.ones(self.N), umax)])

        # Equality + inequality constraints
        lower_bounds = np.hstack([lower_equality_bounds, lower_inequality_bounds])
        upper_bounds = np.hstack([upper_equality_bounds, upper_inequality_bounds])
        return lower_bounds, upper_bounds

    def mpc_control(self, x0: float) -> float:
        """Solve optimal control problem and return the first control action."""
        # Update initial state
        self.bound_lwr[: self.nx] = -x0
        self.bound_upr[: self.nx] = -x0
        self.solver.update_bounds(self.bound_lwr, self.bound_upr)

        # Solve
        res = self.solver.solve()

        # Return the first control action
        return res[0][-self.N * self.nu : -(self.N - 1) * self.nu]

    def my_update(self, t: int) -> None:  # noqa: ARG002
        """Update internal state estimate and control action."""
        # State estimate using LQE
        # x is the physical state estimate
        # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
        # LQE is only needed for x since error_sum and action are known exactly
        x = self._z[0:2]
        x = np.dot(self.AL, x) + np.dot(self.B, self._u) + np.dot(self.L, self.error)
        self._z = np.hstack([x, self.error_sum, self._u])

        # Solve optimization problem
        du = self.mpc_control(self._z)[0]
        self._u += du
