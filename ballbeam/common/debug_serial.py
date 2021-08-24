from time import time, sleep
import struct

import numpy as np
from serial import Serial

from ballbeam.common.extramath import saturate
from ballbeam.common.settings import PORT, BAUD_RATE, DT, DISTANCE_BACKSTOP, TIMEOUT_STEPS, \
    RAD2DEG, SERVO_CMD_REST, SERVO_CMD_MID, SERVO_CMD_MIN, SERVO_CMD_MAX, \
    SERVO_CALIBRATION_COEFFICIENTS, SENSOR_CALIBRATION_COEFFICIENTS, \
    BEAM_ANGLE_MIN, BEAM_ANGLE_MAX


def create_serial(port=None, baud_rate=None, timeout=1):
    if port is None:
        port = PORT
    if baud_rate is None:
        baud_rate = BAUD_RATE
    return Serial(port, baud_rate, timeout=timeout)

# TODO figure out why servo sometimes jumps when starting (or stopping?) the system
# Idea: might be when Serial() or ser.close() is called it sends out some data that gets read by the Arduino and converted to a valid servo command pwm
# Fix: make a state machine in the Arduino control.ino sketch and use a sentinel value e.g. 0 to tell the Arduino to always write servo commmand pwm to rest

ser = create_serial()
# ser.close()
