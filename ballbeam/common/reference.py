import numpy as np

from ballbeam.configurators.configs import CONFIG


class Reference:
    """Base class for reference trajectory generators."""

    def __init__(self) -> None:
        pass

    def setpoint(self, t) -> float:
        return 0.0


class ConstantReference(Reference):
    """A reference at a constant value."""

    def __init__(self, center=0.0) -> None:
        super().__init__()
        self.center = center  # in meters

    def setpoint(self, t):
        return self.center


class PeriodicReference(ConstantReference):
    """A reference that changes periodically with a given waveform."""

    def __init__(self, amplitude=0.050, frequency=0.1, waveform="sine") -> None:
        super().__init__()
        self.amplitude = amplitude  # in meters
        self.frequency = frequency  # in Hz
        self.waveform = waveform

    def setpoint(self, t):
        phase = self.frequency * t * CONFIG.hardware.COMM.DT
        phase_rad = 2 * np.pi * phase
        if self.waveform == "sine":
            deviation = np.sin(phase_rad)
        elif self.waveform == "rounded_square":
            sharpness = 3  # bigger = closer to a square wave, smaller = closer to a sine wave
            deviation = np.tanh(sharpness * np.sin(phase_rad))
        elif self.waveform == "square":
            deviation = np.sign(float(phase % 2 > 1) - 0.5)
        else:
            raise ValueError
        return self.center + self.amplitude * deviation
