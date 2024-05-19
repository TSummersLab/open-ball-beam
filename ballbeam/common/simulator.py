"""Class representing a simulated version of the Open Ball & Beam."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

import numpy as np
import numpy.random as npr

from ballbeam.common.extramath import saturate
from ballbeam.common.system import System
from ballbeam.configurators.configs import CONFIG

if TYPE_CHECKING:
    from ballbeam.common.type_defs import ArrF64

XMIN, XMAX = -0.115, 0.115  # limits of physical position, in meters
YMIN, YMAX = -0.125, 0.125  # limits of realized position measurement, in meters
UMIN, UMAX = (
    CONFIG.hardware.BEAM.ANGLE.MIN * CONFIG.constants.DEG2RAD,
    CONFIG.hardware.BEAM.ANGLE.MAX * CONFIG.constants.DEG2RAD,
)


def step(
    f: Callable[[ArrF64, float], ArrF64],
    x: ArrF64,
    u: float,
    w: ArrF64,
    dt: float | None = None,
    method: str = "rk4",
) -> ArrF64:
    """Compute a single step using a numerical integration scheme.

    Args:
    ----
    f: Callable continuous-time dynamics function that defines the ordinary differential equation (ODE) x_dot = f(x, u).
    x: Current state.
    u: Current action.
    w: Current disturbance. This disturbance is added at the end i.e. after discretization in time.
    dt: Delta time.
    method: Numerical integration method. Can be one of "euler", "rk2", "rk4".
    """
    if dt is None:
        dt = CONFIG.hardware.COMM.DT
    if method == "euler":
        x1 = np.copy(x)
        k1 = f(x1, u)
        x_new = x1 + dt * k1
    elif method == "rk2":
        x1 = np.copy(x)
        k1 = f(x1, u)
        x2 = x1 + dt * k1
        k2 = f(x2, u)
        x_new = x1 + (dt / 2) * (k1 + k2)
    elif method == "rk4":
        x1 = np.copy(x)
        k1 = f(x1, u)
        x2 = x1 + dt * k1 / 2
        k2 = f(x2, u)
        x3 = x2 + dt * k2 / 2
        k3 = f(x3, u)
        x4 = x3 + dt * k3
        k4 = f(x4, u)
        x_new = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    else:
        msg = f'Step method "{method}" is not implemented!'
        raise ValueError(msg)

    # Add disturbance (in discrete-time)
    x_new += w
    return x_new


class SimulatorSystem(System):
    """Class representing a simulation of the Open Ball & Beam."""

    def __init__(
        self,
        x0: ArrF64 | None = None,
        servo_assumption: str = "instant",
        process_noise_scale: float = 1.0,
        sensor_noise_scale: float = 1.0,
        random_seed: int = 0,
    ) -> None:
        """Initialize."""
        # Configuration
        self.config = CONFIG.model

        self.servo_assumption = servo_assumption
        if self.servo_assumption == "instant":
            self.n = 2
            self.W = np.diag([1e-6, 1e-5])  # process noise covariance
        elif self.servo_assumption == "speed_limited":
            self.n = 3
            self.W = np.diag([1e-6, 1e-5, 1e-9])
        else:
            raise ValueError

        self.V = np.diag([1e-6])  # sensor noise covariance

        self.W *= process_noise_scale
        self.V *= sensor_noise_scale

        self.m, self.p = 1, 1

        if x0 is None:
            x0 = np.zeros(self.n)
            x0[0] = XMIN
        self.x = x0

        self.rng = npr.default_rng(random_seed)

    def generate_process_noise(self, *, do_kick: bool = False) -> ArrF64:
        """Generate process noise."""
        noise = self.rng.multivariate_normal(np.zeros(self.n), self.W)
        if do_kick:
            kick_mag = 100 * self.rng.choice([0, 1], p=[0.99, 0.01])
            kick_base = self.rng.multivariate_normal(np.zeros(self.n), self.W)
            kick = kick_mag * kick_base
        else:
            kick = 0
        return noise + kick

    def generate_observation_noise(self) -> ArrF64:
        """Generate observation noise."""
        return self.rng.multivariate_normal(np.zeros(self.p), self.V)[0]

    def f(self, x: ArrF64, u: float) -> ArrF64:
        """Compute the output of the dynamics function i.e. x_dot."""
        # TODO(bgravell): implement the missing term related to \dot{theta}^2  # noqa: TD003, FIX002
        # TODO(bgravell): implement deadband for servo commands that do not exceed 3ms PWM difference  # noqa: TD003, FIX002, E501

        sin_u_angle = np.clip(u, -1.0, 1.0)

        if self.servo_assumption == "instant":
            # This dynamics function assumes the commanded beam angle is achieved instantly
            # x[0]  Ball position
            # x[1]  Ball velocity
            # u     Sine of commanded beam angle

            return np.array([x[1], -self.config.GRAVITY_SCALED * sin_u_angle - self.config.DAMP_SCALED * x[1]])

        if self.servo_assumption == "speed_limited":
            # This dynamics function assumes servo-style tracking of the commanded beam angle
            # x[0]  Ball position (m)
            # x[1]  Ball velocity (m/s)
            # x[2]  Beam angle (rad)
            # u     Sine of commanded beam angle

            u_angle = np.arcsin(sin_u_angle)

            # Make the servo speed function
            def speed_fun(x: float, *, hard: bool = True) -> float:
                """Compute the scaling factor to apply to the motor speed.

                Args:
                ----
                x: The difference between the commanded beam angle and the current beam angle state.
                hard: If true, assumes motor speed instantly achieves max signed value when crossing x=0.
                      If false, assumes motor speed has to transition between limits around x=0.

                Return:
                ------
                Scaling factor of motor speed.
                """
                return float(np.sign(x) if hard else np.tanh(self.config.TRANSITION_RATE * x))

            return np.array(
                [
                    x[1],
                    -self.config.GRAVITY_SCALED * np.sin(x[2]) - self.config.DAMP_SCALED * x[1],
                    self.config.MOTOR_SPEED * speed_fun(u_angle - x[2]),
                ],
            )

        msg = f'Invalid servo assumption "{self.servo_assumption}"'
        raise ValueError(msg)

    def process(self, u: float) -> None:
        """Process a control action."""
        u, self.saturated = saturate(u, UMIN, UMAX)
        w = self.generate_process_noise()
        self.x = step(self.f, self.x, u, w)
        self.x[0] = np.clip(self.x[0], XMIN, XMAX)

    def observe(self) -> float:
        """Collect an observation."""
        v = self.generate_observation_noise()
        y = self.x[0] + v
        return float(np.clip(y, YMIN, YMAX))

    def reset(self, x: ArrF64 | None = None) -> None:
        """Reset the system."""
        if x is None:
            x = np.zeros(self.n)
            x[0] = XMIN
        self.x = x

    def shutdown(self) -> None:
        """Shut down the system."""
        print("")
        print("Shutting down")
