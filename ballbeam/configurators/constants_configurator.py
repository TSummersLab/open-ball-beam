import numpy as np

from ballbeam.configurators.configurators import Configurator


def make_constants_configurator():
    name = 'constants'
    description = "Universal constants"
    data = dict(RAD2DEG=360.0/(2.0*np.pi),
                DEG2RAD=(2.0*np.pi)/360.0,
                STD_GRAVITY_ACCEL=9.80665)  # standard gravitational acceleration in m/s/s
    return Configurator(name, data, description)