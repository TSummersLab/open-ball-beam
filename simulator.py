import numpy as np
import numpy.random as npr

from settings import DT
XMIN, XMAX = -0.115, 0.115  # limits of physical position, in meters
YMIN, YMAX = -0.125, 0.125  # limits of realized position measurement, in meters


def step(f, x, u, w, dt=None, method='rk4'):
    if dt is None:
        dt = DT
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
    def __init__(self, x0=None, gravity=9.81, damp=1e-3):
        self.n, self.m, self.p = 2, 1, 1
        self.gravity = gravity  # meters/second/second
        self.damp = damp  # 1/second
        self.W = np.diag([1e-7, 1e-6])  # process noise covariance
        self.V = np.diag([1e-7])  # sensor noise covariance

        if x0 is None:
            x0 = np.zeros(self.n)
            x0[0] = XMIN
        self.x = x0

    def generate_process_noise(self):
        return npr.multivariate_normal(np.zeros(self.n), self.W)

    def generate_observation_noise(self):
        return npr.multivariate_normal(np.zeros(self.p), self.V)[0]

    def f(self, x, u):
        # x[0]  Ball position
        # x[1]  Ball velocity
        # u     Beam angle
        return np.array([x[1], -self.gravity*np.sin(u) - self.damp])

    def process(self, u, actuation=None):
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
        print('shutting down')
        return
