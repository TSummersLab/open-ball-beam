# The purpose of this script is to generate the configuration objects from the YAML files,
# which can be accessed across various Python scripts
import os

from ballbeam.common.yaml_io import yaml_import
from ballbeam.common.utility import Dict2Obj


this_dir, this_filename = os.path.split(__file__)  # Get path of this file


def make_config_obj(filename):
    return Dict2Obj(yaml_import(os.path.join(this_dir, filename)))


constants_config = make_config_obj('../configuration/constants_config.yaml')
model_config = make_config_obj('../configuration/model_config.yaml')
hardware_config = make_config_obj('../configuration/hardware_config.yaml')
interface_config = make_config_obj('../configuration/interface_config.yaml')
plot_config = make_config_obj('../configuration/plot_config.yaml')
servo_calibration_config = make_config_obj('../configuration/servo_calibration_config.yaml')
sensor_calibration_config = make_config_obj('../configuration/sensor_calibration_config.yaml')
