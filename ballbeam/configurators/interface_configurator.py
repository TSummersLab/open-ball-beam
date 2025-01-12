"""Configurator for interface."""

from ballbeam.configurators.configurators import Configurator


def make_interface_configurator() -> Configurator:
    """Make a configurator for the interface."""

    name = "interface"
    description = "Interface settings"

    # system_type = "Hardware"
    system_type = "Simulator"

    # controller_type = "Random"
    controller_type = "PID"
    # controller_type = "LQG"
    # controller_type = "MSPC"

    # reference_type = "Constant"
    reference_type = "FastSquare"
    # reference_type = "SlowSine"
    # reference_type = "RandomWaves"

    cost_type = "Default"

    monitor_types = ["Print", "Plot"]
    # monitor_types = ["Print"]

    logger_type = "Full"

    data = {
        "system_type": system_type,
        "controller_type": controller_type,
        "reference_type": reference_type,
        "cost_type": cost_type,
        "monitor_types": monitor_types,
        "logger_type": logger_type,
    }

    return Configurator(name, data, description)
