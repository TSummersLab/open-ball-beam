"""Classes to represent the hardware of the Open Ball & Beam."""

from __future__ import annotations

import copy
import struct
from time import sleep, time
from typing import TYPE_CHECKING

import numpy as np
from serial import Serial

from ballbeam.common.extramath import saturate, sparse2dense_coeffs
from ballbeam.common.system import System
from ballbeam.configurators.configs import CONFIG

if TYPE_CHECKING:
    from ballbeam.common.type_defs import ArrF64


class HardwareSystem(System):
    """Class representing a physical Open Ball & Beam."""

    def __init__(self, ser: Serial | None = None, num_init_reads: int = 3) -> None:
        """Initialize."""
        # Configuration
        self.config = CONFIG.hardware
        self.reading_offset = CONFIG.sensor_calibration.READING_OFFSET
        self.servo_coefficients = sparse2dense_coeffs(
            CONFIG.servo_calibration.coefficients,
            CONFIG.servo_calibration.powers,
        )
        self.sensor_coefficients = sparse2dense_coeffs(
            CONFIG.sensor_calibration.coefficients,
            CONFIG.sensor_calibration.powers,
        )

        # Start the serial connection
        if ser is None:
            self.ser = Serial(port=self.config.COMM.PORT, baudrate=self.config.COMM.BAUD_RATE, timeout=1)

        # Check to make sure the hardware is working properly
        for _i in range(num_init_reads):
            raw_line = self.ser.readline()
            line = raw_line.decode("utf-8").rstrip()
            if "Failed" in line:
                raise RuntimeError(line)

        # Initialize variables
        self.observation = self.observe()
        self.timeout = 0
        self.saturated = False
        self.ball_removed = False

    def detect_ball_removed(self) -> tuple[bool, int]:
        """Detect whether the ball has been removed or not.

        Timeout logic:
        If ball has been removed, cease operation.
        If ball has not been removed, resume operation.
        """
        if not self.ball_removed:
            if self.observation > self.config.SENSOR.DISTANCE.BACKSTOP_MM:
                self.timeout += 1
            else:
                self.timeout = 0
        else:  # noqa: PLR5501
            if self.observation <= self.config.SENSOR.DISTANCE.BACKSTOP_MM:
                self.timeout -= 1
            else:
                self.timeout = 2 * self.config.TIMEOUT.STEPS

        self.ball_removed = self.timeout > self.config.TIMEOUT.STEPS
        return self.ball_removed, self.timeout

    def action2actuation(
        self,
        action: float,
        coefficients: list[float] | None = None,
        *,
        linearize: bool = False,
    ) -> int:
        """Convert standard action to a raw actuation.

        Args:
        ----
        action: beam_angle in radians
        coefficients: polynomial coefficients in decreasing power order from d down to 0.
        linearize: Make the linearization assumption that theta = np.sin(theta)

        Return:
        ------
        action, in pwm microseconds for servo
        """
        if self.ball_removed:
            return self.config.SERVO.CMD.MID

        if coefficients is None:
            coefficients = self.servo_coefficients

        # Saturate action into interval [-1, 1] before passing thru np.arcsin()
        # This will convert the action to a beam angle
        action_sat, saturated1 = saturate(action, -1.0, 1.0)

        beam_angle = copy.copy(action) if linearize else np.arcsin(action_sat)

        # Saturate beam angle against the system limits
        beam_angle, saturated2 = saturate(
            beam_angle,
            self.config.BEAM.ANGLE.MIN * CONFIG.constants.DEG2RAD,
            self.config.BEAM.ANGLE.MAX * CONFIG.constants.DEG2RAD,
        )

        # Convert beam angle to an actuation PWM using the servo calibration polynomial coefficients
        x = beam_angle * self.config.BEAM.ANGLE_SCALE
        y = np.polyval(coefficients, x)
        actuation_deviation = int(y / self.config.SERVO.PWM_SCALE)
        actuation = self.config.SERVO.CMD.MID + actuation_deviation

        # Saturate actuation PWM against system limits
        actuation, saturated3 = saturate(actuation, self.config.SERVO.CMD.MIN, self.config.SERVO.CMD.MAX)

        saturated = saturated1 or saturated2 or saturated3

        self.saturated = saturated
        return actuation

    def process(self, action: float) -> None:
        """Process an action."""
        # Check if ball was removed
        self.detect_ball_removed()

        # Convert action to a raw actuation PWM value for the servo
        actuation = self.action2actuation(action)

        # Send the actuation command to serial
        out = struct.pack("h", actuation)
        self.ser.write(out)

    def read_int(self) -> int:
        """Read integer from serial connection."""
        raw_line = self.ser.readline()
        line = raw_line.decode("utf-8").rstrip()
        try:
            val = int(line)
        except Exception:  # noqa: BLE001
            val = 0
        return val

    def reading2observation(self, reading: int, coefficients: list[float] | None = None) -> float:
        """Convert a raw reading to a standard observation.

        Args:
        ----
        reading: reading of sensor distance, in millimeters.
        coefficients: sensor calibration coefficients.

        Return:
        ------
        observation: calibrated distance to ball, in meters
        """
        if coefficients is None:
            coefficients = self.sensor_coefficients

        x = (reading - self.reading_offset) * self.config.SENSOR.READING_SCALE
        y = np.polyval(coefficients, x)

        return (0.001 / self.config.SENSOR.OBSERVATION_SCALE) * y

    def observe(self) -> float:
        """Collect an observation."""
        # Read the measurement value from serial
        reading = self.read_int()

        # Convert raw reading to a standard observation
        self.observation = self.reading2observation(reading)
        return self.observation

    def reset(self, x: ArrF64 | None = None) -> None:  # noqa: ARG002
        """Reset the system."""
        print("Resetting system...")
        time_start = time()
        # give time for ball to roll down
        rest_time_seconds = 2.0
        rest_steps = int(rest_time_seconds / self.config.COMM.DT)
        for _i in range(rest_steps + 1):
            out = struct.pack("h", self.config.SERVO.CMD.REST)
            self.ser.write(out)
            self.observe()
        time_end = time()
        time_elapsed = time_end - time_start
        print("...system reset after resting %.3f seconds" % time_elapsed)

    def shutdown(self) -> None:
        """Shut down the system."""
        print("")
        print("Shutting down...")
        print("")
        self.reset()
        sleep(0.1)
        # Close serial connection
        print("Closing serial connection...")
        if self.ser is not None:
            self.ser.close()
        print("...serial connection closed.")
        sleep(0.1)
        print("...system shut down.")
