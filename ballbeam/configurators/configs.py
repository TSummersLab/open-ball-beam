"""Configuration for ballbeam.

This module loads all YAML config files as objects for use across the library.
"""

import dataclasses
from typing import Any

from ballbeam.common.utility import convert_dict_to_object, ConvertedObject
from ballbeam.common.yaml_io import yaml_import
from ballbeam.static import CONFIGURATION_PATH


def load_config_as_object(filename: str) -> ConvertedObject:
    return convert_dict_to_object(yaml_import(CONFIGURATION_PATH.joinpath(filename)))


@dataclasses.dataclass
class BallBeamConfig:
    constants: Any
    model: Any
    hardware: Any
    interface: Any
    plot: Any
    servo_calibration: Any
    sensor_calibration: Any

    @classmethod
    def from_filesystem(cls) -> "BallBeamConfig":
        config_attrs = [field.name for field in dataclasses.fields(cls)]
        configs = {attr: load_config_as_object(f"{attr}_config.yaml") for attr in config_attrs}
        return cls(**configs)


CONFIG = BallBeamConfig.from_filesystem()
