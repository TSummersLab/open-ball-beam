# from ballbeam.configuration import constants_configurator
# from ballbeam.configuration import model_configurator
# from ballbeam.configuration import hardware_configurator
#
# from ballbeam.configuration import servo_calibration
# from ballbeam.configuration import sensor_calibration
#
# from ballbeam.configuration import plot_configurator
# from ballbeam.configuration import interface_configurator

configurators = ['constants_configurator.py',
                 'model_configurator.py',
                 'hardware_configurator.py',
                 'servo_calibration.py',
                 'sensor_calibration.py',
                 'plot_configurator.py',
                 'interface_configurator.py']

for configurator in configurators:
    exec(open(configurator).read())

