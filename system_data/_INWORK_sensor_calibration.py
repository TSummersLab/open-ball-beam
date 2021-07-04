import sys
import keyboard
from time import time
import struct
import numpy as np
from serial import Serial
from settings import PORT, BAUD_RATE, RAD2DEG, ACTUATION_EQ, ACTUATION_MIN, ACTUATION_MAX, SENSOR_BACKSTOP, TIMEOUT_STEPS

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from controller import Controller, Reference



def create_serial():
    return Serial(PORT, BAUD_RATE, timeout=0.5)


def read_int(ser):
    try:
        raw_line = ser.readline()
        line = raw_line.decode('utf-8').rstrip()
        val = int(line)
    except:
        val = 0
    return val



ser = create_serial()


num_readings = 100
readings = []

for i in range(num_readings):
    # Read the measurement value from serial
    reading = read_int(ser)
    readings.append(reading)


ground_truth = 100  # in millimeters


# TODO use input() to wait for user to enter the ground_truth, then take num_readings and store data in dict, and have an exit key option to stop