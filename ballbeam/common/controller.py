import os

import numpy as np

from ballbeam.common.extramath import mix, saturate
from ballbeam.common.settings import DT, RAD2DEG
from ballbeam.common.pickle_io import pickle_import


class Controller:
    def __init__(self):
        self._u = 0.0  # control action
        self._z = np.zeros(4)  # augmented state estimate

    def update_aux(self, saturated=False, ball_removed=False):
        # arg: saturated, boolean, true if actuator is currently saturated
        # arg: ball_removed, boolean, true if ball is currently removed from beam
        self.saturated = saturated
        self.ball_removed = ball_removed
        return

    def update(self, observation, setpoint, t):
        # arg: observation, in meters
        # arg: setpoint, target observation in meters
        # arg: t, time index since start
        pass

    @property
    def action(self):
        if self.ball_removed:
            self._u = 0.0
        return self._u

    @property
    def state_estimate(self):
        if self.ball_removed:
            self._z = np.zeros(4)
        return self._z


# Sine "controller" that just generates a sine wave
class SineController(Controller):
    def __init__(self, freq=0.3, beam_angle_max_deg=2.0):
        super().__init__()
        self.freq = freq  # in Hz
        self.beam_angle_max_deg = beam_angle_max_deg  # in degrees

    def update(self, observation, setpoint, t):
        beam_angle_max_rad = self.beam_angle_max_deg/RAD2DEG
        self._u = beam_angle_max_rad*np.sin(2*np.pi*self.freq*t*DT)


# PID with exponential smoothing filters
class PIDController(Controller):
    def __init__(self, kp=0.6, ki=0.1, kd=0.3, error_mix=0.6, error_diff_mix=0.2):
        super().__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_mix = error_mix
        self.error_diff_mix = error_diff_mix

        self.error = 0
        self.error_sum = 0
        self.error_last = 0
        self.error_diff = 0

    def update(self, observation, setpoint, t, anti_windup=True):
        new_error = observation - setpoint
        self.error = mix(new_error, self.error, self.error_mix)

        new_error_diff = (new_error - self.error_last)/DT
        self.error_diff = mix(new_error_diff, self.error_diff, self.error_diff_mix)

        u_p = self.kp*self.error
        u_i = self.ki*self.error_sum
        u_d = self.kd*self.error_diff
        self._u = u_p + u_i + u_d

        if anti_windup and self.saturated:
            pass
        else:
            self.error_sum += self.error*DT
        self.error_last = self.error


# Linear quadratic Gaussian with integral feedback & control difference penalization
class LQGController(Controller):
    def __init__(self, controller_data_path=None):
        super().__init__()
        self.error = 0
        self.error_sum = 0

        if controller_data_path is None:
            this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl
            controller_data_path = os.path.join(this_dir, 'controller_data.pickle')

        controller_data = pickle_import(controller_data_path)

        self.A = controller_data['A']
        self.B = controller_data['B']
        self.C = controller_data['C']
        self.K = controller_data['K']
        self.L = controller_data['L']

        self.AL = self.A + np.dot(self.L[:, None], self.C[None, :])

    def update(self, observation, setpoint, t, anti_windup=True):
        if self.ball_removed:
            self.error = 0
            self.error_sum = 0
        else:
            # Control action using LQR
            du = np.dot(self.K, self._z)
            self._u += du

            # Update error
            self.error = observation - setpoint

            # Update error sum
            if anti_windup and self.saturated:
                pass
            else:
                self.error_sum += self.error

            # State estimate using LQE
            # x is the physical state estimate
            # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
            x = self._z[0:2]
            x = np.dot(self.AL, x)
            x += np.dot(self.B, self._u)
            x -= np.dot(self.L, self.error)
            self._z = np.hstack([x, np.array([self.error_sum, self._u])])


class MPCController(Controller):
    def __init__(self, controller_data_path=None):
        super().__init__()

        # TODO implement
        raise NotImplementedError

        self.error = 0
        self.error_sum = 0

        if controller_data_path is None:
            this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl
            controller_data_path = os.path.join(this_dir, 'controller_data.pickle')

        controller_data = pickle_import(controller_data_path)

        self.A = controller_data['A']
        self.B = controller_data['B']
        self.C = controller_data['C']

    def update(self, observation, setpoint, t, anti_windup=True):
        # Form optimization problem

        # Solve optimization problem

        # Use the first control input in the plan and throw the rest away
        # u = u_plan[0]
        # return u

        pass