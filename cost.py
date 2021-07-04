import numpy as np


class Cost:
    def __init__(self, e0=0, u0=0):
        self.e = np.copy(e0)  # output error
        self.u = np.copy(u0)  # action

    def take(self, e, u):
        c_error = 10*np.abs(e)
        # c_error = 1.0*float(np.abs(e) > 0.020)  # penalize errors above threshold equally
        c_action = 8*np.abs(u)
        c_action_diff = 3*np.abs(u-self.u)
        c = c_error + c_action + c_action_diff
        self.e = e
        self.u = u
        c_dict = dict(c=c, c_error=c_error, c_action=c_action, c_action_diff=c_action_diff)
        return c_dict
