import numpy as np

from ballbeam.common.yaml_io import yaml_export


# Physical constants
data = dict(RAD2DEG=360.0/(2.0*np.pi),
            DEG2RAD=(2.0*np.pi)/360.0,
            STD_GRAVITY_ACCEL=9.80665)  # standard gravitational acceleration in m/s/s

yaml_export(data, 'constants_config.yaml')
