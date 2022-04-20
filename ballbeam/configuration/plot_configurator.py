import os

from ballbeam.common.yaml_io import yaml_import, yaml_export
from ballbeam.common.utility import Dict2Obj

hardware_config = Dict2Obj(yaml_import('hardware_config.yaml'))


# Choose the data categories to plot
# SHOW_KEYS = ['position', 'state_estimate', 'action', 'cost']
SHOW_KEYS = ['position', 'state_estimate', 'action']
# SHOW_KEYS = ['position', 'cost']
# SHOW_KEYS = ['position']  # disabling more reduces plotting overhead
# SHOW_KEYS = None  # use SHOW_KEYS = None to disable plotting altogether

SCROLL = True
LENGTH_SECONDS = 4.0  # do not make this too big or else plotting will be too slow and affect control
LENGTH = int(LENGTH_SECONDS/hardware_config.COMM.DT)  # convert from seconds to # of steps
ANTIALIAS = False
WINDOW_SIZE = [800, 600]  # [width, height] in pixels


data = dict(SHOW_KEYS=SHOW_KEYS,
            SCROLL=SCROLL,
            LENGTH_SECONDS=LENGTH_SECONDS,
            LENGTH=LENGTH,
            ANTIALIAS=ANTIALIAS,
            WINDOW_SIZE=WINDOW_SIZE)

yaml_export(data, 'plot_config.yaml')
