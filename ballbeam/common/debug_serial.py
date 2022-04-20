from time import time, sleep
import struct

import numpy as np
from serial import Serial

from ballbeam.common.extramath import saturate
from ballbeam.configuration.configs import hardware_config



# TODO figure out why servo sometimes jumps when starting (or stopping?) the system
# Idea: might be when Serial() or ser.close() is called it sends out some data that gets read by the Arduino and converted to a valid servo command pwm
# Fix: make a state machine in the Arduino control.ino sketch and use a sentinel value e.g. 0 to tell the Arduino to always write servo commmand pwm to rest

ser = Serial(hardware_config.COMM.PORT, hardware_config.COMM.BAUD_RATE, timeout=1)
# ser.close()
