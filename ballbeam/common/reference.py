"""Classes representing reference trajectories."""

import secrets
from functools import partial

import numpy as np
from scipy.signal import square, sawtooth

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


class PeriodicReference(Reference):
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
            deviation = -np.cos(phase_rad)
        elif self.waveform == "rounded_square":
            sharpness = 3  # bigger = closer to a square wave, smaller = closer to a sine wave
            deviation = np.tanh(sharpness * np.sin(phase_rad))
        elif self.waveform == "square":
            deviation = np.sign(float(phase % 2 > 1) - 0.5)
        else:
            raise ValueError
        return self.amplitude * deviation


class SlowSineReference(PeriodicReference):
    """A slow sine reference trajectory."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__(amplitude=0.040, frequency=0.10, waveform="sine")


class FastSquareReference(PeriodicReference):
    """A fast square reference trajectory."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__(amplitude=0.030, frequency=0.20, waveform="square")


class RandomWavesReference(Reference):
    """A random waves reference trajectory."""

    def get_wavefunc(self, waveform: str):
        if waveform == "sine":
            return np.sin
        elif waveform == "square":
            return square
        elif waveform == "triangle":
            return partial(sawtooth, width=0.5)
        elif waveform == "sawtooth up":
            return partial(sawtooth, width=1.0)
        elif waveform == "sawtooth down":
            return partial(sawtooth, width=0.0)

        raise ValueError

    def __init__(self) -> None:
        super().__init__()
        self.seed = secrets.randbits(128)
        self.rng = np.random.default_rng(self.seed)
        setpoint_history_chunks = []
        for i in range(1000):
            if i == 0:
                # Give the system a chance to get stabilized in a neutral position for 10 seconds.
                duration = 10.0
                amplitude = 0.0
                waveform = "sine"
                frequency = 1.0
                phase = 0.0
            else:
                duration = self.rng.uniform(low=1.0, high=5.0)
                amplitude = self.rng.uniform(low=0.0, high=0.025)
                waveform = self.rng.choice(["sine", "square", "triangle", "sawtooth up", "sawtooth down"], p=[0.5, 0.1, 0.2, 0.1, 0.1])
                frequency = 10**(self.rng.uniform(low=-1.0, high=0.0))
                phase = self.rng.uniform(low=0.0, high=1.0)

            wavefunc = self.get_wavefunc(waveform)

            t = np.arange(0, duration, CONFIG.hardware.COMM.DT)

            chunk = amplitude * wavefunc(2 * np.pi * (frequency * t + phase))

            setpoint_history_chunks.append(chunk)
        self.setpoint_history = np.hstack(setpoint_history_chunks)

    def setpoint(self, t: int) -> float:
        """Compute the setpoint at time index t."""
        return self.setpoint_history[t]



# Register all classes with this map
# TODO(bgravell): Refactor to decorate each concrete class with this registration # noqa: TD003, FIX002
REFERENCE_CLASS_MAP = {
    "Constant": ConstantReference,
    "SlowSine": SlowSineReference,
    "FastSquare": FastSquareReference,
    "RandomWaves": RandomWavesReference,
}
