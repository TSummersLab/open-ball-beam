# The purpose of this script is to generate the configuration objects from the YAML files,
# which can be accessed across various Python scripts
import os

from ballbeam.static import CONFIGURATION_PATH
from ballbeam.common.yaml_io import yaml_import
from ballbeam.common.utility import Dict2Obj


def make_config_obj(filename):
    return Dict2Obj(yaml_import(CONFIGURATION_PATH.joinpath(filename)))


constants_config = make_config_obj('constants_config.yaml')
model_config = make_config_obj('model_config.yaml')
hardware_config = make_config_obj('hardware_config.yaml')
interface_config = make_config_obj('interface_config.yaml')
plot_config = make_config_obj('plot_config.yaml')
servo_calibration_config = make_config_obj('servo_calibration_config.yaml')
sensor_calibration_config = make_config_obj('sensor_calibration_config.yaml')
