import os

from ballbeam.common.yaml_io import yaml_import, yaml_export

this_dir, this_filename = os.path.split(__file__)  # Get path of this file


########################################################################################################################
# Arduino communications configuration

# Communications
PORT = 'COM7'
# PORT = "COM"+input("Enter a COM port number:\n")

BAUD_RATE = 115200

# Update rates
DT = 0.020  # sampling time in seconds
UPDATE_RATE = 1.0/DT  # in Hz

comm_data = dict(PORT=PORT,
                 BAUD_RATE=BAUD_RATE,
                 DT=DT,
                 UPDATE_RATE=UPDATE_RATE)


########################################################################################################################
# Servo configuration
# Servo limits for ANNIMOS DS3218MG
SERVO_CMD_MID = 1500  # midpoint servo pwm in microseconds
SERVO_CMD_MIN = SERVO_CMD_MID - 500  # safe minimum servo pwm in microseconds
SERVO_CMD_MAX = SERVO_CMD_MID + 500  # safe maximum servo pwm in microseconds
SERVO_CMD_REST = SERVO_CMD_MID + 50  # rest servo pwm, tilt down slightly so ball rolls to end stop

PWM_SCALE = 0.001

servo_calibration_coefficients_path = os.path.join(this_dir, 'servo_calibration_coefficients.yaml')

servo_data = dict(CMD=dict(MID=SERVO_CMD_MID,
                           MIN=SERVO_CMD_MIN,
                           MAX=SERVO_CMD_MAX,
                           REST=SERVO_CMD_REST),
                  PWM_SCALE=PWM_SCALE,
                  CALIBRATION=yaml_import(servo_calibration_coefficients_path))


########################################################################################################################
# Beam configuration
BEAM_ANGLE_MIN = -4.0  # in degrees
BEAM_ANGLE_MAX = 4.0  # in degrees

BEAM_ANGLE_SCALE = 10.0

beam_data = dict(ANGLE=dict(MIN=BEAM_ANGLE_MIN,
                                 MAX=BEAM_ANGLE_MAX),
                 ANGLE_SCALE=BEAM_ANGLE_SCALE)


########################################################################################################################
# Sensor configuration
DISTANCE_MID = 105  # midpoint of beam rails, in millimeters (250mm rails, each end embedded in 20mm deep rail holders, leaving 210mm exposed, so midpoint is 210mm/2 = 105mm)
DISTANCE_BACKSTOP = 90  # max distance from midpoint, in millimeters

READING_OFFSET = 142.454545  # get this value from sensor_calibration.py as the raw sensor reading when ball physically at DISTANCE_MID
READING_SCALE = 0.01
OBSERVATION_SCALE = 0.01

sensor_calibration_coefficients_path = os.path.join(this_dir, 'sensor_calibration_coefficients.yaml')

sensor_data = dict(DISTANCE=dict(MID=DISTANCE_MID,
                                 BACKSTOP=DISTANCE_BACKSTOP),
                   READING_OFFSET=READING_OFFSET,
                   READING_SCALE=READING_SCALE,
                   OBSERVATION_SCALE=OBSERVATION_SCALE,
                   CALIBRATION=yaml_import(sensor_calibration_coefficients_path))


########################################################################################################################
# Timeout configuration
TIMEOUT_TIME = 0.100  # in seconds
TIMEOUT_STEPS = int(TIMEOUT_TIME/DT)

timeout_data = dict(TIME=TIMEOUT_TIME,
                    STEPS=TIMEOUT_STEPS)


########################################################################################################################
data = dict(COMM=comm_data,
            SERVO=servo_data,
            BEAM=beam_data,
            SENSOR=sensor_data,
            TIMEOUT=timeout_data)

yaml_export(data, 'hardware_config.yaml')
