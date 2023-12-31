"""Classes and script to create all configurations for ballbeam."""
from __future__ import annotations

from typing import TYPE_CHECKING, Any

from ballbeam.common.utility import ConvertedObject, convert_dict_to_object
from ballbeam.common.yaml_io import yaml_export, yaml_import
from ballbeam.static import CONFIGURATION_PATH

if TYPE_CHECKING:
    from pathlib import Path


class Configurator:
    """A class for creating configurations."""

    def __init__(self, name: str, data: Any, description: str | None = None) -> None:
        """Initialize."""
        self.name = name
        self.data = data
        self.description = description

        self.file_suffix = "config"
        self.file_extension = ".yaml"
        self.file_separator = "_"
        self.dir = CONFIGURATION_PATH

    @property
    def filename(self) -> str:
        """Return the name of the file where the configuation is stored."""
        return f"{self.name}{self.file_separator}{self.file_suffix}{self.file_extension}"

    @property
    def path(self) -> Path:
        """Return the path to the file where the configuation is stored."""
        return self.dir.joinpath(self.filename)

    def dump(self) -> None:
        """Write the configuration."""
        yaml_export(self.dir, self.filename, self.data)

    def load(self) -> Any:
        """Load the configuration."""
        return yaml_import(self.filename)

    @property
    def data_obj(self) -> ConvertedObject:
        """Return the data converted to an object."""
        return convert_dict_to_object(self.data)


def configure_calibrate_all(*, skip_mpc_setup: bool = True) -> None:
    """Run all configuration and calibration routines."""
    from ballbeam.calibrators.sensor_calibrator import make_sensor_calibration_configurator
    from ballbeam.calibrators.servo_calibrator import make_servo_calibration_configurator
    from ballbeam.configurators.constants_configurator import make_constants_configurator
    from ballbeam.configurators.hardware_configurator import make_hardware_configurator
    from ballbeam.configurators.interface_configurator import make_interface_configurator
    from ballbeam.configurators.model_configurator import make_model_configurator
    from ballbeam.configurators.plot_configurator import make_plot_configurator

    constants_configurator = make_constants_configurator()
    model_configurator = make_model_configurator(constants_configurator)
    hardware_configurator = make_hardware_configurator()
    interface_configurator = make_interface_configurator()
    plot_configurator = make_plot_configurator(hardware_configurator)

    servo_calibration_configurator = make_servo_calibration_configurator(constants_configurator, hardware_configurator)
    sensor_calibration_configurator = make_sensor_calibration_configurator(
        constants_configurator,
        hardware_configurator,
    )

    configurators = [
        constants_configurator,
        model_configurator,
        hardware_configurator,
        interface_configurator,
        plot_configurator,
        servo_calibration_configurator,
        sensor_calibration_configurator,
    ]

    # Export
    for configurator in configurators:
        configurator.dump()

    # Controllers
    from ballbeam.configurators.controller.lqg.design import main as lqg_main
    from ballbeam.configurators.controller.mpc.design import main as mpc_main
    from ballbeam.configurators.controller.pid.design import main as pid_main

    pid_main()
    lqg_main()
    if not skip_mpc_setup:
        # Skip the MPC setup since it takes a while (includes code generation - slow!)
        mpc_main()


if __name__ == "__main__":
    configure_calibrate_all()
