import os

from ballbeam.static import CONFIGURATION_PATH
from ballbeam.common.yaml_io import yaml_import, yaml_export
from ballbeam.common.utility import convert_dict_to_object


class Configurator:
    def __init__(self, name, data, description=None):
        self.name = name
        self.data = data
        self.description = description

        self.file_suffix = 'config'
        self.file_extension = '.yaml'
        self.file_separator = '_'
        self.dir = CONFIGURATION_PATH

    @property
    def filename(self):
        return self.name + self.file_separator + self.file_suffix + self.file_extension

    @property
    def path(self):
        return os.path.join(self.dir, self.filename)

    def dump(self):
        yaml_export(self.dir, self.filename, self.data)

    def load(self):
        return yaml_import(self.filename)

    @property
    def data_obj(self):
        return convert_dict_to_object(self.data)


def configure_calibrate_all():
    from ballbeam.configurators.constants_configurator import make_constants_configurator
    from ballbeam.configurators.model_configurator import make_model_configurator
    from ballbeam.configurators.hardware_configurator import make_hardware_configurator
    from ballbeam.configurators.interface_configurator import make_interface_configurator
    from ballbeam.configurators.plot_configurator import make_plot_configurator

    from ballbeam.calibrators.servo_calibrator import make_servo_calibration_configurator
    from ballbeam.calibrators.sensor_calibrator import make_sensor_calibration_configurator

    constants_configurator = make_constants_configurator()
    model_configurator = make_model_configurator(constants_configurator)
    hardware_configurator = make_hardware_configurator()
    interface_configurator = make_interface_configurator()
    plot_configurator = make_plot_configurator(hardware_configurator)

    servo_calibration_configurator = make_servo_calibration_configurator(constants_configurator, hardware_configurator)
    sensor_calibration_configurator = make_sensor_calibration_configurator(constants_configurator, hardware_configurator)

    configurators = [constants_configurator,
                     model_configurator,
                     hardware_configurator,
                     interface_configurator,
                     plot_configurator,
                     servo_calibration_configurator,
                     sensor_calibration_configurator]

    # Export
    for configurator in configurators:
        configurator.dump()

    # Controllers
    from ballbeam.configurators.controller.pid.design import main as pid_main
    from ballbeam.configurators.controller.lqg.design import main as lqg_main
    from ballbeam.configurators.controller.mpc.design import main as mpc_main
    pid_main()
    lqg_main()
    # mpc_main()


if __name__ == '__main__':
    configure_calibrate_all()
