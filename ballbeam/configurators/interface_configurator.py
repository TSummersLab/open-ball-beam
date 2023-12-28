from ballbeam.configurators.configurators import Configurator


def make_interface_configurator():
    name = "interface"
    description = "Interface settings"

    # system_type = "Simulator"
    system_type = "Hardware"

    # controller_type = "Null"
    # controller_type = "Sine"
    # controller_type = "PID"
    controller_type = "LQG"
    # controller_type = "MPC"

    # reference_type = "Constant"
    # reference_type = "SlowSine"
    reference_type = "FastSquare"

    cost_type = "Default"

    data = {
        "system_type": system_type,
        "controller_type": controller_type,
        "reference_type": reference_type,
        "cost_type": cost_type,
    }
    return Configurator(name, data, description)
