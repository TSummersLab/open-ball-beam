"""Classes representing reference trajectories."""

import numpy as np

from ballbeam.configurators.configs import CONFIG


class Reference:
    """Base class for reference trajectory generators."""

    def setpoint(self, t: int) -> float:  # noqa: ARG002
        """Compute the setpoint at time index t."""
        return 0.0


class ConstantReference(Reference):
    """A reference at a constant value."""

    def __init__(self, center: float = 0.0) -> None:
        """Initialize."""
        super().__init__()
        self.center = center  # in meters

    def setpoint(self, t: int) -> float:  # noqa: ARG002
        """Compute the setpoint at time index t."""
        return self.center


class PeriodicReference(ConstantReference):
    """A reference that changes periodically with a given waveform."""

    def __init__(self, amplitude: float = 0.050, frequency: float = 0.1, waveform: str = "sine") -> None:
        """Initialize."""
        super().__init__()
        self.amplitude = amplitude  # in meters
        self.frequency = frequency  # in Hz
        self.waveform = waveform

    def setpoint(self, t: int) -> float:
        """Compute the setpoint at time index t."""
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


class SlowSineReference(PeriodicReference):
    """A slow sine reference trajectory."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__(amplitude=0.050, frequency=0.10, waveform="sine")


class FastSquareReference(PeriodicReference):
    """A fast square reference trajectory."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__(amplitude=0.030, frequency=0.20, waveform="square")


# Register all classes with this map
# TODO(bgravell): Refactor to decorate each concrete class with this registration # noqa: TD003, FIX002
REFERENCE_CLASS_MAP = {
    "Constant": ConstantReference,
    "SlowSine": SlowSineReference,
    "FastSquare": FastSquareReference,
}
