"""Configurator for interface."""

from ballbeam.configurators.configurators import Configurator


def make_interface_configurator() -> Configurator:
    """Make a configurator for the interface.

    system_type can be one of:
    "Simulator"
    "Hardware"

    controller_type can be one of:
    "Null"
    "Sine"
    "PID"
    "LQG"
    "MPC"

    reference_type can be one of:
    "Constant"
    "SlowSine"
    "FastSquare"

    cost_type can be one of:
    "Default"

    monitor_types can be a list consisting of any of:
    "Print"
    "Plot"

    logger_type can be one of:
    "None"
    "Full"
    """
    name = "interface"
    description = "Interface settings"

    system_type = "Hardware"
    controller_type = "LQG"
    reference_type = "FastSquare"
    cost_type = "Default"
    monitor_types = ["Print", "Plot"]
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
