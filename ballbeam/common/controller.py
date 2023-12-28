import numpy as np

from ballbeam.common.extramath import mix
from ballbeam.common.pickle_io import pickle_import
from ballbeam.configurators.configs import CONFIG
from ballbeam.static import CONFIGURATION_PATH


class Controller:
    def __init__(self, error_mix=1.0, error_diff_mix=1.0, anti_windup=False):
        # Settings
        self.error_mix = error_mix  # 1.0 is no smoothing, << 1.0 is lots of smoothing
        self.error_diff_mix = error_diff_mix  # 1.0 is no smoothing, << 1.0 is lots of smoothing
        self.anti_windup = anti_windup

        # Initialize self state
        self._u = 0.0  # control action
        self._z = np.zeros(4)  # augmented state estimate

        self.error = 0
        self.error_sum = 0
        self.error_last = 0
        self.error_diff = 0

        self.saturated = False
        self.ball_removed = False

    def update_aux(self, saturated=False, ball_removed=False):
        # arg: saturated, boolean, true if actuator is currently saturated
        # arg: ball_removed, boolean, true if ball is currently removed from beam
        self.saturated = saturated
        self.ball_removed = ball_removed
        return

    def error_update(self, observation, setpoint):
        # Update error based on current observation and setpoint
        error_new = observation - setpoint
        self.error = mix(error_new, self.error, self.error_mix)

        error_diff_new = (error_new - self.error_last)/CONFIG.hardware.COMM.DT
        self.error_diff = mix(error_diff_new, self.error_diff, self.error_diff_mix)

        # This is anti-windup, functionality shared across multiple controllers
        if self.anti_windup and self.saturated:
            pass
        else:
            self.error_sum += self.error*CONFIG.hardware.COMM.DT

        # Store the last error
        self.error_last = self.error
        return

    def my_update(self, t):
        # Dummy
        return

    def update(self, observation, setpoint, t):
        # arg: observation, in meters
        # arg: setpoint, target observation in meters
        # arg: t, time index since start

        self.error_update(observation, setpoint)

        # This is deactivation upon ball removal, functionality shared across multiple controllers
        if self.ball_removed:
            self.error = 0
            self.error_sum = 0
        else:
            self.my_update(t)

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

    def my_update(self, t):
        beam_angle_max_rad = self.beam_angle_max_deg / CONFIG.constants.RAD2DEG
        self._u = beam_angle_max_rad*np.sin(2 * np.pi * self.freq * t * CONFIG.hardware.COMM.DT)


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

    def my_update(self, t):
        # Control
        u_p = self.kp*self.error
        u_i = self.ki*self.error_sum
        u_d = self.kd*self.error_diff
        self._u = u_p + u_i + u_d

        # State estimate
        self._z = np.array([self.error, self.error_diff, self.error_sum, self._u])


# Linear quadratic Gaussian (LQG) with integral feedback & control difference penalization
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

    def my_update(self, t):
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


# Model predictive control (MPC) - same as LQG but with state and input constraint handling
class MPCController(Controller):
    def __init__(self, controller_params_path=None):
        super().__init__()

        from ballbeam.common import emosqp  # Must match the python_ext_name='emosqp' from prob.codegen() in configurators/controller/mpc/design.py
        self.solver = emosqp

        if controller_params_path is None:
            controller_params_path = CONFIGURATION_PATH.joinpath('controller', 'mpc', 'controller_params.pickle')
        controller_params = pickle_import(controller_params_path)

        self.A = controller_params['A']
        self.B = controller_params['B']
        self.C = controller_params['C']
        self.K = controller_params['K']
        self.L = controller_params['L']

        self.AL = self.A - np.dot(self.L[:, None], self.C[None, :])

        design_data_path = CONFIGURATION_PATH.joinpath('controller', 'mpc', 'design_data.pickle')
        controller_design_data = pickle_import(design_data_path)

        A4, B4, Q4, QN4, R4 = [controller_design_data['controller'][key] for key in ['A4', 'B4', 'Q4', 'QN4', 'R4']]
        xmin, xmax, umin, umax = [controller_design_data['controller'][key] for key in ['xmin', 'xmax', 'umin', 'umax']]
        self.N = controller_design_data['N']
        self.nx, self.nu = B4.shape
        self.bound_lwr, self.bound_upr = self.make_bounds(xmin, xmax, umin, umax)

    def make_bounds(self, xmin, xmax, umin, umax):
        # Input and state constraints
        # Equality constraints
        leq = np.hstack([np.zeros(self.nx), np.zeros(self.N*self.nx)])
        ueq = leq
        # Inequality constraints
        lineq = np.hstack([np.kron(np.ones(self.N + 1), xmin), np.kron(np.ones(self.N), umin)])
        uineq = np.hstack([np.kron(np.ones(self.N + 1), xmax), np.kron(np.ones(self.N), umax)])
        # Equality + inequality constraints
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])
        return l, u

    def mpc_control(self, x0):
        # Update initial state
        self.bound_lwr[:self.nx] = -x0
        self.bound_upr[:self.nx] = -x0
        self.solver.update_bounds(self.bound_lwr, self.bound_upr)

        # Solve
        res = self.solver.solve()
        return res[0][-self.N*self.nu:-(self.N-1)*self.nu]  # Return the first control input

    def my_update(self, t):
        # State estimate using LQE
        # x is the physical state estimate
        # z is the augmented state estimate, which is the physical state augmented with the error_sum and action
        # LQE is only needed for x since error_sum and action are known exactly
        x = self._z[0:2]
        x = np.dot(self.AL, x) + np.dot(self.B, self._u) + np.dot(self.L, self.error)
        self._z = np.hstack([x, self.error_sum, self._u])

        # Solve optimization problem
        # du = mpc_control(self._z, self.l_base, self.u_base, self.nx, self.nu, self.N)[0]
        du = self.mpc_control(self._z)[0]
        self._u += du
