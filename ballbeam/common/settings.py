import os

import numpy as np

from ballbeam.common.pickle_io import pickle_import

########################################################################################################################
# Path info
this_dir, this_filename = os.path.split(__file__)  # Get path of data.pkl

########################################################################################################################
# Physical constants
RAD2DEG = 360.0/(2.0*np.pi)
DEG2RAD = (2.0*np.pi)/360.0

STD_GRAVITY_ACCEL = 9.80665  # standard gravitational acceleration in m/s/s, needed for IMU calibration

########################################################################################################################
# Hardware settings

# # PORT = 'COM3'
PORT = 'COM6'
# PORT = "COM"+input("Enter a COM port number:\n")

BAUD_RATE = 115200

DT = 0.020  # sampling time in seconds
UPDATE_RATE = 1.0/DT  # in Hz

# Servo limits for ANNIMOS DS3218MG
SERVO_CMD_MID = 1500  # midpoint servo pwm in microseconds
SERVO_CMD_MIN = SERVO_CMD_MID - 500  # safe minimum servo pwm in microseconds
SERVO_CMD_MAX = SERVO_CMD_MID + 500  # safe maximum servo pwm in microseconds
SERVO_CMD_REST = SERVO_CMD_MID + 50  # rest servo pwm, tilt down slightly so ball rolls to end stop

PWM_SCALE = 0.001
BEAM_ANGLE_SCALE = 10.0

BEAM_ANGLE_MIN = -4.0  # in degrees
BEAM_ANGLE_MAX = 4.0  # in degrees

data_path = os.path.join(this_dir, '..', 'system_data/servo_calibration_coefficients.pkl')
SERVO_CALIBRATION_COEFFICIENTS = pickle_import(data_path)

data_path = os.path.join(this_dir, '..', 'system_data/sensor_calibration_coefficients.pkl')
SENSOR_CALIBRATION_COEFFICIENTS = pickle_import(data_path)

DISTANCE_MID = 105  # midpoint of beam rails, in millimeters (250mm rails, each end embedded in 20mm deep rail holders, leaving 210mm exposed, so midpoint is 210mm/2 = 105mm)
DISTANCE_BACKSTOP = 0.090  # max distance from midpoint, in meters

READING_OFFSET = 142.454545  # get this value from sensor_calibration.py as the raw sensor reading when ball physically at DISTANCE_MID
READING_SCALE = 0.01
OBSERVATION_SCALE = 0.01

TIMEOUT_TIME = 0.100  # in seconds
TIMEOUT_STEPS = int(TIMEOUT_TIME/DT)

########################################################################################################################
# Model constants
GRAVITY = STD_GRAVITY_ACCEL  # meters/second/second
BALL_RADIUS = 0.01905  # meters
BALL_MASS = 0.039  # kilograms
MASS_SCALE = 1.0 + (2.0/5.0)  # scaling constant of mass resulting from the ball's rotational inertia. This value is for a solid sphere since J = (2/5)*m*R**2
DAMP = 0.01  # 1/second
MOTOR_SPEED = (60/0.16)*DEG2RAD  # rad/sec
TRANSITION_RATE = 100.0

########################################################################################################################
# Plot settings

# choose the data categories to plot
# SHOW_KEYS = ['position', 'state_estimate', 'action', 'cost']
SHOW_KEYS = ['position', 'state_estimate', 'action']
# SHOW_KEYS = ['position', 'cost']
# SHOW_KEYS = ['position']  # disabling more reduces plotting overhead
# SHOW_KEYS = None  # use SHOW_KEYS = None to disable plotting altogether

PLOT_SCROLL = True
PLOT_LENGTH_SECONDS = 4.0  # do not make this too big or else plotting will be too slow and affect control
PLOT_LENGTH = int(PLOT_LENGTH_SECONDS/DT)  # convert from seconds to # of steps
PLOT_ANTIALIAS = False
PLOT_WINDOW_SIZE = [800, 600]  # [width, height] in pixels
