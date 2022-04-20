import os

from ballbeam.common.yaml_io import yaml_import
from ballbeam.common.utility import Dict2Obj


this_dir, this_filename = os.path.split(__file__)  # Get path of this file


def make_config_obj(filename):
    return Dict2Obj(yaml_import(os.path.join(this_dir, filename)))


# The purpose of this script is to generate the configuration objects from the YAML files,
# which can be accessed across various files

constants_config = make_config_obj('constants_config.yaml')
model_config = make_config_obj('model_config.yaml')
hardware_config = make_config_obj('hardware_config.yaml')
servo_calibration_config = make_config_obj('servo_calibration.yaml')
sensor_calibration_config = make_config_obj('sensor_calibration.yaml')
plot_config = make_config_obj('plot_config.yaml')
interface_config = make_config_obj('interface_config.yaml')
