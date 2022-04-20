import numpy as np

from ballbeam.common.yaml_io import yaml_import, yaml_export


system_type = 'Simulator'
# system_type = 'Hardware'

# controller_type = 'Null'
# controller_type = 'Sine'
# controller_type = 'PID'
controller_type = 'LQG'
# controller_type = 'MPC'

# reference_type = 'Constant'
# reference_type = 'SlowSine'
reference_type = 'FastSquare'


data = dict(system_type=system_type,
            controller_type=controller_type,
            reference_type=reference_type)

yaml_export(data, 'interface_config.yaml')
