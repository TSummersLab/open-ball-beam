"""Configuration for ballbeam.

This module loads all YAML config files as objects for use across the library.
"""

import dataclasses

from ballbeam.static import CONFIGURATION_PATH
from ballbeam.common.yaml_io import yaml_import
from ballbeam.common.utility import convert_dict_to_object


def load_config_as_object(filename):
    return convert_dict_to_object(yaml_import(CONFIGURATION_PATH.joinpath(filename)))


@dataclasses.dataclass
class BallBeamConfig:
    constants: object
    model: object
    hardware: object
    interface: object
    plot: object
    servo_calibration: object
    sensor_calibration: object

    @classmethod
    def from_filesystem(cls):
        config_attrs = [field.name for field in dataclasses.fields(cls)]
        configs = {
            attr: load_config_as_object(f"{attr}_config.yaml")
            for attr in config_attrs
        }
        return cls(**configs)


CONFIG = BallBeamConfig.from_filesystem()
