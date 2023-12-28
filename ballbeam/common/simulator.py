import numpy as np
import numpy.random as npr

from ballbeam.common.extramath import saturate
from ballbeam.configurators.configs import CONFIG


XMIN, XMAX = -0.115, 0.115  # limits of physical position, in meters
YMIN, YMAX = -0.125, 0.125  # limits of realized position measurement, in meters
UMIN, UMAX = CONFIG.hardware.BEAM.ANGLE.MIN*CONFIG.constants.DEG2RAD, CONFIG.hardware.BEAM.ANGLE.MAX*CONFIG.constants.DEG2RAD


def step(f, x, u, w, dt=None, method='rk4'):
    if dt is None:
        dt = CONFIG.hardware.COMM.DT
    if method == 'euler':
        x1 = np.copy(x)
        k1 = f(x1, u)
        x_new = x1 + dt*k1
    elif method == 'rk2':
        x1 = np.copy(x)
        k1 = f(x1, u)
        x2 = x1 + dt*k1
        k2 = f(x2, u)
        x_new = x1 + (dt/2)*(k1 + k2)
    elif method == 'rk4':
        x1 = np.copy(x)
        k1 = f(x1, u)
        x2 = x1 + dt*k1/2
        k2 = f(x2, u)
        x3 = x2 + dt*k2/2
        k3 = f(x3, u)
        x4 = x3 + dt*k3
        k4 = f(x4, u)
        x_new = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    else:
        raise ValueError('Choose a different step method.')
    x_new += w
    return x_new


class Simulator:
    def __init__(self, x0=None, servo_assumption='instant', process_noise_scale=1.0, sensor_noise_scale=1.0):
        # Configuration
        self.config = CONFIG.model

        self.servo_assumption = servo_assumption
        if self.servo_assumption == 'instant':
            self.n = 2
            self.W = np.diag([1e-6, 1e-5])  # process noise covariance
        elif self.servo_assumption == 'speed_limited':
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

        self.saturated = False
        self.ball_removed = False

    def generate_process_noise(self, do_kick=False):
        noise = npr.multivariate_normal(np.zeros(self.n), self.W)
        if do_kick:
            kick_mag = 100*npr.choice([0, 1], p=[0.99, 0.01])
            kick_base = npr.multivariate_normal(np.zeros(self.n), self.W)
            kick = kick_mag*kick_base
        else:
            kick = 0
        return noise + kick

    def generate_observation_noise(self):
        return npr.multivariate_normal(np.zeros(self.p), self.V)[0]

    def f(self, x, u):
        # TODO implement the missing term related to \dot{theta}^2
        # TODO implement deadband for servo commands that do not exceed 3ms PWM difference

        sin_u_angle = np.clip(u, -1, 1)

        if self.servo_assumption == 'instant':
            # This dynamics function assumes the commanded beam angle is achieved instantly
            # x[0]  Ball position
            # x[1]  Ball velocity
            # u     Sine of commanded beam angle

            return np.array([x[1],
                             -self.config.GRAVITY_SCALED*sin_u_angle - self.config.DAMP_SCALED*x[1]])

        elif self.servo_assumption == 'speed_limited':
            # This dynamics function assumes servo-style tracking of the commanded beam angle

            # x[0]  Ball position (m)
            # x[1]  Ball velocity (m/s)
            # x[2]  Beam angle (rad)
            # u     Sine of commanded beam angle

            u_angle = np.arcsin(sin_u_angle)

            # Make the servo speed function
            def speed_fun(x, hard=True):
                return np.sign(x) if hard else np.tanh(self.config.TRANSITION_RATE*x)

            return np.array([x[1],
                             -self.config.GRAVITY_SCALED*np.sin(x[2]) - self.config.DAMP_SCALED*x[1],
                             self.config.MOTOR_SPEED*speed_fun(u_angle - x[2])])

    def process(self, u):
        u, self.saturated = saturate(u, UMIN, UMAX)
        w = self.generate_process_noise()
        self.x = step(self.f, self.x, u, w)
        self.x[0] = np.clip(self.x[0], XMIN, XMAX)
        return self.x

    def observe(self):
        v = self.generate_observation_noise()
        y = self.x[0] + v
        y = np.clip(y, YMIN, YMAX)
        return y

    def reset(self, x):
        self.x = x

    def shutdown(self):
        print('')
        print('shutting down')
