import numpy as np
from settings import DT, RAD2DEG
from pickle_io import pickle_import

from extramath import mix, saturate


class Controller:
    def __init__(self):
        self.u = 0.0  # control action
        self.z = np.zeros(4)  # augmented state estimate

    def update(self, observation, setpoint, t, saturated=False, ball_removed=False):
        # arg: observation, in meters
        # arg: setpoint, target observation in meters
        # arg: t, time index since start
        # arg: saturated, boolean, true if actuator is currently saturated
        # arg: ball_removed, boolean, true if ball is currently removed from beam
        pass

    @property
    def action(self):
        return self.u

    @property
    def state_estimate(self):
        return self.z


# Silly "controller" that just generates a sine wave
class SillyController(Controller):
    def __init__(self):
        super().__init__()
        self.freq = 0.5  # in Hz
        self.beam_angle_max_deg = 5  # in degrees

    def update(self, observation, setpoint, t, saturated=False, ball_removed=False):
        beam_angle_max_rad = self.beam_angle_max_deg/RAD2DEG
        self.u = beam_angle_max_rad*np.sin(2*np.pi*self.freq*t*DT)


# PID with exponential smoothing filters and partial control dropout near setpoint
class PIDController(Controller):
    def __init__(self, kp=0.5, ki=0.01, kd=20.0, error_mix=0.6, error_diff_mix=0.2, u_mix=0.5, dropout_width=0.3):
        super().__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_mix = error_mix
        self.error_diff_mix = error_diff_mix
        self.u_mix = u_mix
        self.dropout_width = dropout_width

        self.error = 0
        self.error_sum = 0
        self.error_last = 0
        self.error_diff = 0

    def update(self, observation, setpoint, t, saturated=False, ball_removed=False, anti_windup=True):
        new_error = observation - setpoint
        self.error = mix(new_error, self.error, self.error_mix)
        new_error_diff = new_error - self.error_last
        self.error_diff = mix(new_error_diff, self.error_diff, self.error_diff_mix)
        action_p = self.kp*self.error
        action_i = self.ki*self.error_sum
        action_d = self.kd*self.error_diff
        u_pid = action_p + action_i + action_d
        fade_scale = min(np.abs(np.tanh(10*self.error/self.dropout_width)), 1)
        u_pid *= fade_scale
        self.u = mix(u_pid, self.u, self.u_mix)

        if anti_windup and saturated:
            pass
        else:
            self.error_sum += self.error
        self.error_last = self.error


# Linear quadratic Gaussian with integral feedback & control difference penalization
class LQGController(Controller):
    def __init__(self, controller_data_path=None):
        super().__init__()
        self.error = 0
        self.error_sum = 0

        if controller_data_path is None:
            import os
            this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl
            controller_data_path = os.path.join(this_dir, 'controller_data.pickle')

        controller_data = pickle_import(controller_data_path)

        self.A = controller_data['A']
        self.B = controller_data['B']
        self.C = controller_data['C']
        self.K = controller_data['K']
        self.L = controller_data['L']

        self.AL = self.A + np.dot(self.L[:, None], self.C[None, :])

    def update(self, observation, setpoint, t, saturated=False, ball_removed=False, anti_windup=True):
        if ball_removed:
            self.error = 0
            self.error_sum = 0

            self.u = 0.0
            self.z = np.zeros(4)
        else:
            # Control action using LQR
            du = np.dot(self.K, self.z)
            self.u += du

            # Update error
            self.error = observation - setpoint

            # Update error sum
            if anti_windup and saturated:
                pass
            else:
                self.error_sum += self.error

            # State estimate using LQE
            # x is the physical state estimate
            # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
            x = self.z[0:2]
            x = np.dot(self.AL, x)
            x += np.dot(self.B, self.u)
            x -= np.dot(self.L, self.error)
            self.z = np.hstack([x, np.array([self.error_sum, self.u])])
