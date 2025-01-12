from dataclasses import dataclass
import numpy.typing as npt


@dataclass
class Example:
    y: npt.NDArray
    u: npt.NDArray
    s: npt.NDArray


def example_from_yus(example_idx, Y, U, S):

    return Example(
        Y[example_idx],
        U[example_idx],
        S[example_idx],
    )