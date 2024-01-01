"""Configurator for plotting."""

from ballbeam.configurators.configurators import Configurator


def make_plot_configurator(hardware_configurator: Configurator) -> Configurator:
    """Make a configurator for plotting.

    SHOW_KEYS is a list of keys to the data categories to plot:
    "position"
    "state_estimate"
    "action"
    "cost"

    Plotting overhead will increase as more quantities are plotted.
    Leave SHOW_KEYS as an empty list to disable plotting altogether.
    """
    name = "plot"
    description = "Plot settings"

    hardware_config = hardware_configurator.data_obj

    # Choose the data categories to plot
    SHOW_KEYS = ["position", "state_estimate", "action"]

    SCROLL = True
    # Do not make this too big or else plotting will be too slow and affect control
    LENGTH_SECONDS = 4.0
    # Convert from seconds to # of steps
    LENGTH_STEPS = int(LENGTH_SECONDS / hardware_config.COMM.DT)  # type: ignore[attr-defined]
    ANTIALIAS = False
    WINDOW_SIZE = [800, 600]  # [width, height] in pixels

    data = {
        "SHOW_KEYS": SHOW_KEYS,
        "SCROLL": SCROLL,
        "LENGTH_SECONDS": LENGTH_SECONDS,
        "LENGTH_STEPS": LENGTH_STEPS,
        "ANTIALIAS": ANTIALIAS,
        "WINDOW_SIZE": WINDOW_SIZE,
    }
    return Configurator(name, data, description)
