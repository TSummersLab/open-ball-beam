from time import time, sleep
import struct

import numpy as np
from serial import Serial


from ballbeam.common.extramath import saturate, sparse2dense_coeffs
from ballbeam.configuration.configs import constants_config, hardware_config, servo_calibration_config, sensor_calibration_config


class Hardware:
    def __init__(self, ser=None, num_init_reads=3):
        # Configuration
        self.config = hardware_config
        self.servo_coefficients = sparse2dense_coeffs(servo_calibration_config.coefficients,
                                                      servo_calibration_config.powers)
        self.sensor_coefficients = sparse2dense_coeffs(sensor_calibration_config.coefficients,
                                                       sensor_calibration_config.powers)

        # Initialize variables
        self.observation = None
        self.timeout = 0
        self.saturated = False
        self.ball_removed = False

        # Start the serial connection
        if ser is None:
            self.ser = Serial(port=self.config.COMM.PORT,
                              baudrate=self.config.COMM.BAUD_RATE,
                              timeout=1)

        # Check to make sure the hardware is working properly
        for i in range(num_init_reads):
            raw_line = self.ser.readline()
            line = raw_line.decode('utf-8').rstrip()
            if 'Failed' in line:
                raise RuntimeError(line)

    def detect_ball_removed(self):
        # Timeout logic: detect if ball has been removed and cease operation if so, resume if not
        if not self.ball_removed:
            if self.observation > self.config.SENSOR.DISTANCE.BACKSTOP_MM:
                self.timeout += 1
            else:
                self.timeout = 0
        else:
            if self.observation <= self.config.SENSOR.DISTANCE.BACKSTOP_MM:
                self.timeout -= 1
            else:
                self.timeout = 2*self.config.TIMEOUT.STEPS
        self.ball_removed = self.timeout > self.config.TIMEOUT.STEPS
        return self.ball_removed, self.timeout

    def action2actuation(self, action, coefficients=None):
        # Convert standard action to a raw actuation
        # arg: action, beam_angle in radians
        # arg: coefficients, polynomial coefficients in decreasing power order from d down to 0
        # return action, in pwm microseconds for servo
        if self.ball_removed:
            return self.config.SERVO.CMD.MID
        else:
            if coefficients is None:
                coefficients = self.servo_coefficients

            # # This line makes the linearization assumption that theta = np.sin(theta)
            # beam_angle = action

            # Saturate action into interval [-1, 1] before passing thru np.arcsin()
            # This will convert the action to a beam angle
            action_sat, saturated1 = saturate(action, -1.0, 1.0)

            beam_angle = np.arcsin(action_sat)

            # Saturate beam angle against the system limits
            beam_angle, saturated2 = saturate(beam_angle,
                                              self.config.BEAM.ANGLE.MIN*constants_config.DEG2RAD,
                                              self.config.BEAM.ANGLE.MAX*constants_config.DEG2RAD)

            # Convert beam angle to an actuation PWM using the servo calibration polynomial coefficients
            x = beam_angle*self.config.BEAM.ANGLE_SCALE
            y = np.polyval(coefficients, x)
            actuation_deviation = int(y/self.config.SERVO.PWM_SCALE)
            actuation = self.config.SERVO.CMD.MID + actuation_deviation

            # Saturate actuation PWM against system limits
            actuation, saturated3 = saturate(actuation, self.config.SERVO.CMD.MIN, self.config.SERVO.CMD.MAX)

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
            coefficients = self.sensor_coefficients

        x = (reading - self.config.SENSOR.READING_OFFSET)*self.config.SENSOR.READING_SCALE
        y = np.polyval(coefficients, x)
        observation = (0.001/self.config.SENSOR.OBSERVATION_SCALE)*y
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
        rest_steps = int(rest_time_seconds/self.config.COMM.DT)
        for i in range(rest_steps):
            out = struct.pack('h', self.config.SERVO.CMD.REST)
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
