from ballbeam.configurators.configurators import Configurator


def make_plot_configurator(hardware_configurator):
    name = 'plot'
    description = "Plot settings"

    hardware_config = hardware_configurator.data_obj

    # Choose the data categories to plot
    # SHOW_KEYS = ['position', 'state_estimate', 'action', 'cost']
    SHOW_KEYS = ['position', 'state_estimate', 'action']
    # SHOW_KEYS = ['position', 'cost']
    # SHOW_KEYS = ['position']  # disabling more reduces plotting overhead
    # SHOW_KEYS = None  # use SHOW_KEYS = None to disable plotting altogether

    SCROLL = True
    LENGTH_SECONDS = 4.0  # do not make this too big or else plotting will be too slow and affect control
    LENGTH_STEPS = int(LENGTH_SECONDS/hardware_config.COMM.DT)  # convert from seconds to # of steps
    ANTIALIAS = False
    WINDOW_SIZE = [800, 600]  # [width, height] in pixels

    data = dict(SHOW_KEYS=SHOW_KEYS,
                SCROLL=SCROLL,
                LENGTH_SECONDS=LENGTH_SECONDS,
                LENGTH_STEPS=LENGTH_STEPS,
                ANTIALIAS=ANTIALIAS,
                WINDOW_SIZE=WINDOW_SIZE)
    return Configurator(name, data, description)
