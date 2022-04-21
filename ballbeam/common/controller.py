import os

import numpy as np

from ballbeam.common.mpc_design import get_mpc_design_data
# from ballbeam.common.mpc import make_problem, mpc_control
from ballbeam.common.mpc_osqp_codegen import make_base_bounds, mpc_control

from ballbeam.common.extramath import mix
from ballbeam.common.pickle_io import pickle_import
from ballbeam.configurators.configs import constants_config, hardware_config
from ballbeam.static import CONFIGURATION_PATH


class Controller:
    def __init__(self):
        self._u = 0.0  # control action
        self._z = np.zeros(4)  # augmented state estimate
        self.saturated = False
        self.ball_removed = False

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
        return

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
        beam_angle_max_rad = self.beam_angle_max_deg/constants_config.RAD2DEG
        self._u = beam_angle_max_rad*np.sin(2*np.pi*self.freq*t*hardware_config.DT)


# PID with exponential smoothing filters
class PIDController(Controller):
    def __init__(self, controller_params_path=None):
        super().__init__()
        if controller_params_path is None:
            controller_params_path = CONFIGURATION_PATH.joinpath('controller', 'pid', 'controller_params.pickle')
        controller_params = pickle_import(controller_params_path)

        self.kp = controller_params['kp']
        self.ki = controller_params['ki']
        self.kd = controller_params['kd']

        self.error_mix = controller_params['error_mix']
        self.error_diff_mix = controller_params['error_diff_mix']

        self.anti_windup = controller_params['anti_windup']

        self.error = 0
        self.error_sum = 0
        self.error_last = 0
        self.error_diff = 0

    def update(self, observation, setpoint, t):
        error_new = observation - setpoint
        self.error = mix(error_new, self.error, self.error_mix)

        error_diff_new = (error_new - self.error_last)/hardware_config.DT
        self.error_diff = mix(error_diff_new, self.error_diff, self.error_diff_mix)

        # Control
        u_p = self.kp*self.error
        u_i = self.ki*self.error_sum
        u_d = self.kd*self.error_diff
        self._u = u_p + u_i + u_d

        # State estimate
        self._z = np.array([self.error, self.error_diff, self.error_sum, self._u])

        if self.anti_windup and self.saturated:
            pass
        else:
            self.error_sum += self.error*hardware_config.DT
        self.error_last = self.error


# Linear quadratic Gaussian with integral feedback & control difference penalization
class LQGController(Controller):
    def __init__(self, controller_params_path=None):
        super().__init__()
        if controller_params_path is None:
            controller_params_path = CONFIGURATION_PATH.joinpath('controller', 'lqg', 'controller_params.pickle')
        controller_params = pickle_import(controller_params_path)

        self.A = controller_params['A']
        self.B = controller_params['B']
        self.C = controller_params['C']
        self.K = controller_params['K']
        self.L = controller_params['L']

        self.AL = self.A - np.dot(self.L[:, None], self.C[None, :])

        self.error = 0
        self.error_sum = 0

    # def update(self, observation, setpoint, t, anti_windup=True):
    #     if self.ball_removed:
    #         self.error = 0
    #         self.error_sum = 0
    #     else:
    #         # Control action using LQR
    #         du = np.dot(self.K, self._z)
    #         self._u += du
    #
    #         # Update error
    #         self.error = observation - setpoint
    #
    #         # Update error sum
    #         if anti_windup and self.saturated:
    #             pass
    #         else:
    #             self.error_sum += self.error
    #
    #         # State estimate using LQE
    #         # x is the physical state estimate
    #         # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
    #         x = self._z[0:2]
    #         x = np.dot(self.AL, x) + np.dot(self.B, self._u) - np.dot(self.L, self.error)
    #         self._z = np.hstack([x, np.array([self.error_sum, self._u])])

    def update(self, observation, setpoint, t, anti_windup=True):
        if self.ball_removed:
            self.error = 0
            self.error_sum = 0
        else:
            # Update error based on current observation and setpoint
            self.error = observation - setpoint

            # Update error sum
            if anti_windup and self.saturated:
                pass
            else:
                self.error_sum += self.error*hardware_config.COMM.DT

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


# class MPCController(Controller):
#     def __init__(self, controller_params_path=None):
#         super().__init__()
#         self.error = 0
#         self.error_sum = 0
#
#         if controller_params_path is None:
#             this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl
#             controller_params_path = os.path.join(this_dir, 'controller_params.pickle')
#
#         controller_params = pickle_import(controller_params_path)
#
#         self.A = controller_params['A']
#         self.B = controller_params['B']
#         self.C = controller_params['C']
#         self.K = controller_params['K']
#         self.L = controller_params['L']
#
#         self.AL = self.A - np.dot(self.L[:, None], self.C[None, :])
#
#         # Form optimization problem
#         controller_params_path = os.path.join(this_dir, 'controller_design_data.pickle')
#         controller_design_data = pickle_import(controller_params_path)
#
#         A4, B4, Q4, R4 = [controller_design_data[key] for key in ['A', 'B', 'Q', 'R']]
#         QN4 = 2*Q4
#
#         xmax = np.array([1.0, 10.0, 100.0, 0.08])  # position (m), velocity (m/s), integral of position (m), control effort (rad)
#         xmin = -xmax
#         umax = np.array([1.0])  # control difference (rad)
#         umin = -umax
#
#         # Prediction horizon
#         N = 10
#
#         self.x_init, self.u_var, self.prob = make_problem(A4, B4, Q4, QN4, R4, xmin, xmax, umin, umax, N)
#
#     def update(self, observation, setpoint, t, anti_windup=True):
#         if self.ball_removed:
#             self.error = 0
#             self.error_sum = 0
#         else:
#             # Update error based on current observation and setpoint
#             self.error = observation - setpoint
#
#             # Update error sum
#             if anti_windup and self.saturated:
#                 pass
#             else:
#                 self.error_sum += self.error*hardware_config.DT
#
#             # State estimate using LQE
#             # x is the physical state estimate
#             # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
#             # LQE is only needed for x since error_sum and action are known exactly
#             x = self._z[0:2]
#             x = np.dot(self.AL, x) + np.dot(self.B, self._u) + np.dot(self.L, self.error)
#             self._z = np.hstack([x, self.error_sum, self._u])
#
#             # Solve optimization problem
#             du = mpc_control(self._z, self.x_init, self.u_var, self.prob)[0]
#             self._u += du



class MPCController(Controller):
    def __init__(self, controller_params_path=None):
        super().__init__()
        self.error = 0
        self.error_sum = 0

        if controller_params_path is None:
            this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl
            controller_params_path = os.path.join(this_dir, 'controller_params.pickle')

        controller_params = pickle_import(controller_params_path)

        self.A = controller_params['A']
        self.B = controller_params['B']
        self.C = controller_params['C']
        self.K = controller_params['K']
        self.L = controller_params['L']

        self.AL = self.A - np.dot(self.L[:, None], self.C[None, :])

        controller_params_path = os.path.join(this_dir, 'controller_design_data.pickle')
        controller_design_data = pickle_import(controller_params_path)

        A4, B4, Q4, R4 = [controller_design_data[key] for key in ['A', 'B', 'Q', 'R']]

        xmin, xmax, umin, umax, N = get_mpc_design_data()
        self.N = N

        nx, nu = B4.shape
        self.nx, self.nu = nx, nu

        l_base, u_base = make_base_bounds(nx, nu, xmin, xmax, umin, umax, N)

        self.l_base, self.u_base = l_base, u_base

    def update(self, observation, setpoint, t, anti_windup=True):
        if self.ball_removed:
            self.error = 0
            self.error_sum = 0
        else:
            # Update error based on current observation and setpoint
            self.error = observation - setpoint

            # Update error sum
            if anti_windup and self.saturated:
                pass
            else:
                self.error_sum += self.error*hardware_config.DT

            # State estimate using LQE
            # x is the physical state estimate
            # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
            # LQE is only needed for x since error_sum and action are known exactly
            x = self._z[0:2]
            x = np.dot(self.AL, x) + np.dot(self.B, self._u) + np.dot(self.L, self.error)
            self._z = np.hstack([x, self.error_sum, self._u])

            # Solve optimization problem
            du = mpc_control(self._z, self.l_base, self.u_base, self.nx, self.nu, self.N)[0]
            self._u += du
