import numpy as np
from pickle_io import pickle_import
import os

this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl

PORT = 'COM5'
BAUD_RATE = 115200

RAD2DEG = 360.0/(2.0*np.pi)
DEG2RAD = (2.0*np.pi)/360.0

DT = 0.020  # sampling time in seconds
UPDATE_RATE = 1.0/DT  # in Hz

# # for Hitec HS 311
# SERVO_CMD_MID = 1500  # midpoint servo pwm in microseconds
# SERVO_CMD_MIN = 1000  # safe minimum servo pwm in microseconds
# SERVO_CMD_MAX = 2000  # safe maximum servo pwm in microseconds

# for Miuzei DS3218MG
SERVO_CMD_MID = 1540  # midpoint servo pwm in microseconds
SERVO_CMD_MIN = SERVO_CMD_MID-333  # safe minimum servo pwm in microseconds
SERVO_CMD_MAX = SERVO_CMD_MID+333  # safe maximum servo pwm in microseconds
SERVO_CMD_REST = SERVO_CMD_MID+30

# SERVO_CALIBRATION_COEFFICIENTS = pickle_import('system_data/servo_calibration_coefficients.pkl')
data_path = os.path.join(this_dir, 'system_data/servo_calibration_coefficients.pkl')
SERVO_CALIBRATION_COEFFICIENTS = pickle_import(data_path)


SENSOR_MIDPOINT = 0.140  # mid sensor reading, in meters
SENSOR_BACKSTOP = 0.272 - SENSOR_MIDPOINT  # max sensor reading, in meters

TIMEOUT_TIME = 0.1  # in seconds
TIMEOUT_STEPS = int(TIMEOUT_TIME/DT)

# choose the data categories to plot
# SHOW_KEYS = None  # use SHOW_KEYS = None to disable plotting
# SHOW_KEYS = ['position', 'state_estimate', 'action', 'cost']
SHOW_KEYS = ['position', 'state_estimate', 'action']
# SHOW_KEYS = ['position', 'cost']
# SHOW_KEYS = ['position']  # disabling more reduces plotting overhead

PLOT_SCROLL = True
PLOT_LENGTH_SECONDS = 5.0  # do not make this too big or else plotting will be too slow and affect control
PLOT_ANTIALIAS = False
PLOT_WINDOW_SIZE = [1000, 1000]
