import os

from ballbeam.common.yaml_io import yaml_import
from ballbeam.common.utility import Dict2Obj


this_dir, this_filename = os.path.split(__file__)  # Get path of this file


constants_config = Dict2Obj(yaml_import(os.path.join(this_dir, 'constants_config.yaml')))
model_config = Dict2Obj(yaml_import(os.path.join(this_dir, 'model_config.yaml')))
hardware_config = Dict2Obj(yaml_import(os.path.join(this_dir, 'hardware_config.yaml')))
plot_config = Dict2Obj(yaml_import(os.path.join(this_dir, 'plot_config.yaml')))
