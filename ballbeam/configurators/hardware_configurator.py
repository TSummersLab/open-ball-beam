from ballbeam.configurators.configurators import Configurator


def make_hardware_configurator():
    name = 'hardware'
    description = "Hardware settings"

    # Arduino configuration

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

    # Servo configuration
    # Servo limits for ANNIMOS DS3218MG
    SERVO_CMD_MID = 1500  # midpoint servo pwm in microseconds
    SERVO_CMD_MIN = SERVO_CMD_MID - 500  # safe minimum servo pwm in microseconds
    SERVO_CMD_MAX = SERVO_CMD_MID + 500  # safe maximum servo pwm in microseconds
    SERVO_CMD_REST = SERVO_CMD_MID + 50  # rest servo pwm, tilt down slightly so ball rolls to end stop

    PWM_SCALE = 0.001

    servo_data = dict(CMD=dict(MID=SERVO_CMD_MID,
                               MIN=SERVO_CMD_MIN,
                               MAX=SERVO_CMD_MAX,
                               REST=SERVO_CMD_REST),
                      PWM_SCALE=PWM_SCALE)

    # Beam configuration
    BEAM_ANGLE_MIN = -4.0  # in degrees
    BEAM_ANGLE_MAX = 4.0  # in degrees

    BEAM_ANGLE_SCALE = 10.0

    beam_data = dict(ANGLE=dict(MIN=BEAM_ANGLE_MIN,
                                     MAX=BEAM_ANGLE_MAX),
                     ANGLE_SCALE=BEAM_ANGLE_SCALE)

    # Sensor configuration
    DISTANCE_MID = 105  # midpoint of beam rails, in millimeters (250mm rails, each end embedded in 20mm deep rail holders, leaving 210mm exposed, so midpoint is 210mm/2 = 105mm)
    DISTANCE_BACKSTOP = 90  # max distance from midpoint, in millimeters
    DISTANCE_BACKSTOP_MM = 0.001*DISTANCE_BACKSTOP

    READING_SCALE = 0.01
    OBSERVATION_SCALE = 0.01

    sensor_data = dict(DISTANCE=dict(MID=DISTANCE_MID,
                                     BACKSTOP=DISTANCE_BACKSTOP,
                                     BACKSTOP_MM=DISTANCE_BACKSTOP_MM),
                       READING_SCALE=READING_SCALE,
                       OBSERVATION_SCALE=OBSERVATION_SCALE)

    # Timeout configuration
    TIMEOUT_TIME = 0.100  # in seconds
    TIMEOUT_STEPS = int(TIMEOUT_TIME/DT)

    timeout_data = dict(TIME=TIMEOUT_TIME,
                        STEPS=TIMEOUT_STEPS)

    data = dict(COMM=comm_data,
                SERVO=servo_data,
                BEAM=beam_data,
                SENSOR=sensor_data,
                TIMEOUT=timeout_data)
    return Configurator(name, data, description)
