"""Configurator for hardware."""

from ballbeam.configurators.configurators import Configurator

DEFAULT_COM_PORT = "COM6"


def make_hardware_configurator(*, collect_from_user_input: bool = False) -> Configurator:
    """Make a configurator for hardware."""
    name = "hardware"
    description = "Hardware settings"

    # Arduino configuration

    # Communications
    if collect_from_user_input:
        com_port_number = input("Enter a COM port number:\n")
        PORT = f"COM{com_port_number}"
    else:
        PORT = DEFAULT_COM_PORT

    BAUD_RATE = 115200

    # Update rates
    DT = 0.020  # sampling time in seconds
    UPDATE_RATE = 1.0 / DT  # in Hz

    comm_data = {"PORT": PORT, "BAUD_RATE": BAUD_RATE, "DT": DT, "UPDATE_RATE": UPDATE_RATE}

    # Servo configuration
    # Servo limits for ANNIMOS DS3218MG
    SERVO_CMD_MID = 1500  # midpoint servo pwm in microseconds
    SERVO_CMD_MIN = SERVO_CMD_MID - 500  # safe minimum servo pwm in microseconds
    SERVO_CMD_MAX = SERVO_CMD_MID + 500  # safe maximum servo pwm in microseconds
    SERVO_CMD_REST = SERVO_CMD_MID + 50  # rest servo pwm, tilt down slightly so ball rolls to end stop

    PWM_SCALE = 0.001

    servo_data = {
        "CMD": {"MID": SERVO_CMD_MID, "MIN": SERVO_CMD_MIN, "MAX": SERVO_CMD_MAX, "REST": SERVO_CMD_REST},
        "PWM_SCALE": PWM_SCALE,
    }

    # Beam configuration
    BEAM_ANGLE_MIN = -4.0  # in degrees
    BEAM_ANGLE_MAX = 4.0  # in degrees

    BEAM_ANGLE_SCALE = 10.0

    beam_data = {"ANGLE": {"MIN": BEAM_ANGLE_MIN, "MAX": BEAM_ANGLE_MAX}, "ANGLE_SCALE": BEAM_ANGLE_SCALE}

    # Sensor configuration

    # Midpoint of beam rails, in millimeters
    # - 250mm rails, each end embedded in 20mm deep rail holders
    # - Leaves 210mm exposed
    # - So midpoint is 210mm/2 = 105mm
    DISTANCE_MID = 105
    # Max distance from midpoint, first in millimeters, then converted to meters
    DISTANCE_BACKSTOP = 90
    DISTANCE_BACKSTOP_M = 0.001 * DISTANCE_BACKSTOP

    READING_SCALE = 0.01
    OBSERVATION_SCALE = 0.01

    sensor_data = {
        "DISTANCE": {"MID": DISTANCE_MID, "BACKSTOP": DISTANCE_BACKSTOP, "BACKSTOP_MM": DISTANCE_BACKSTOP_M},
        "READING_SCALE": READING_SCALE,
        "OBSERVATION_SCALE": OBSERVATION_SCALE,
    }

    # Timeout configuration
    TIMEOUT_TIME = 0.100  # in seconds
    TIMEOUT_STEPS = int(TIMEOUT_TIME / DT)

    timeout_data = {"TIME": TIMEOUT_TIME, "STEPS": TIMEOUT_STEPS}

    data = {"COMM": comm_data, "SERVO": servo_data, "BEAM": beam_data, "SENSOR": sensor_data, "TIMEOUT": timeout_data}
    return Configurator(name, data, description)
