from time import time, sleep
import struct

import numpy as np
from serial import Serial

from ballbeam.common.extramath import saturate
from ballbeam.common.settings import PORT, BAUD_RATE, DT, DISTANCE_BACKSTOP, TIMEOUT_STEPS, \
    READING_OFFSET, READING_SCALE, OBSERVATION_SCALE, \
    RAD2DEG, DEG2RAD, SERVO_CMD_REST, SERVO_CMD_MID, SERVO_CMD_MIN, SERVO_CMD_MAX, \
    SERVO_CALIBRATION_COEFFICIENTS, SENSOR_CALIBRATION_COEFFICIENTS, \
    BEAM_ANGLE_MIN, BEAM_ANGLE_MAX, BEAM_ANGLE_SCALE, PWM_SCALE


def create_serial(port=None, baud_rate=None, timeout=1):
    if port is None:
        port = PORT
    if baud_rate is None:
        baud_rate = BAUD_RATE
    return Serial(port, baud_rate, timeout=timeout)


class Hardware:
    def __init__(self, ser=None, num_init_reads=3):
        # Initialize variables
        self.observation = None
        self.timeout = 0
        self.saturated = False
        self.ball_removed = False

        # Start the serial connection
        if ser is None:
            self.ser = create_serial()

        # Check to make sure the hardware is working properly
        for i in range(num_init_reads):
            raw_line = self.ser.readline()
            line = raw_line.decode('utf-8').rstrip()
            if 'Failed' in line:
                raise RuntimeError(line)

    def detect_ball_removed(self):
        # Timeout logic: detect if ball has been removed and cease operation if so, resume if not
        if not self.ball_removed:
            if self.observation > DISTANCE_BACKSTOP:
                self.timeout += 1
            else:
                self.timeout = 0
        else:
            if self.observation <= DISTANCE_BACKSTOP:
                self.timeout -= 1
            else:
                self.timeout = 2*TIMEOUT_STEPS
        self.ball_removed = self.timeout > TIMEOUT_STEPS
        return self.ball_removed, self.timeout

    def action2actuation(self, action, coefficients=None):
        # Convert standard action to a raw actuation
        # arg: action, beam_angle in radians
        # arg: coefficients, polynomial coefficients in decreasing power order from d down to 0
        # return action, in pwm microseconds for servo
        if self.ball_removed:
            return SERVO_CMD_MID
        else:
            if coefficients is None:
                coefficients = SERVO_CALIBRATION_COEFFICIENTS

            # # This line makes the linearization assumption that theta = np.sin(theta)
            # beam_angle = action

            # Saturate action into interval [-1, 1] before passing thru np.arcsin()
            # This will convert the action to a beam angle
            action_sat, saturated1 = saturate(action, -1.0, 1.0)

            beam_angle = np.arcsin(action_sat)

            # Saturate beam angle against the system limits
            beam_angle, saturated2 = saturate(beam_angle, BEAM_ANGLE_MIN*DEG2RAD, BEAM_ANGLE_MAX*DEG2RAD)

            # Convert beam angle to an actuation PWM using the servo calibration polynomial coefficients
            x = beam_angle*BEAM_ANGLE_SCALE
            y = np.polyval(coefficients, x)
            actuation_deviation = int(y/PWM_SCALE)
            actuation = SERVO_CMD_MID + actuation_deviation

            # Saturate actuation PWM against system limits
            actuation, saturated3 = saturate(actuation, SERVO_CMD_MIN, SERVO_CMD_MAX)

            saturated = saturated1 or saturated2 or saturated3

            self.saturated = saturated
            return actuation

    def process(self, action):
        # Check if ball was removed
        self.detect_ball_removed()

        # Convert action to a raw actuation PWM value for the servo
        actuation = self.action2actuation(action)

        # Send the actuation command to serial
        out = struct.pack('h', actuation)
        self.ser.write(out)
        return

    def read_int(self):
        raw_line = self.ser.readline()
        line = raw_line.decode('utf-8').rstrip()
        try:
            val = int(line)
        except:
            val = 0
        return val

    def reading2observation(self, reading, coefficients=None):
        # Convert a raw reading to a standard observation
        # arg: reading, in millimeters
        # return: observation, in meters
        if coefficients is None:
            coefficients = SENSOR_CALIBRATION_COEFFICIENTS

        x = (reading - READING_OFFSET)*READING_SCALE
        y = np.polyval(coefficients, x)
        observation = (0.001/OBSERVATION_SCALE)*y
        return observation

    def observe(self):
        # Read the measurement value from serial
        reading = self.read_int()

        # Convert raw reading to a standard observation
        self.observation = self.reading2observation(reading)
        return self.observation

    def reset(self, x=None):
        print('Resetting system...', end='')
        time_start = time()
        # give time for ball to roll down
        rest_time_seconds = 2.0
        rest_steps = int(rest_time_seconds/DT)
        for i in range(rest_steps):
            out = struct.pack('h', SERVO_CMD_REST)
            self.ser.write(out)
            self.observe()
        time_end = time()
        time_elapsed = time_end - time_start
        print('system reset after resting %.3f seconds' % time_elapsed)
        return

    def shutdown(self):
        print('')
        self.reset()
        print('Shutting down')
        sleep(0.1)
        # Close serial connection
        if self.ser is not None:
            self.ser.close()
        sleep(0.1)
        return
