"""Extra math functions."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from ballbeam.common.type_defs import ArrF64


def mix(a: Any, b: Any, x: Any) -> Any:
    """Compute convex combination of a and b with mixing parameter x.

    x = 0 -> b
    x = 1 -> a
    """
    return x * a + (1 - x) * b


def saturate(x: float, xmin: float, xmax: float) -> tuple[float, bool]:
    """Saturate the input value x to bounds xmin and xmax.

    Similar to np.clip, but in addition returns a bool that indicates whether clipping occurred.
    """
    if x > xmax:
        return xmax, True

    if x < xmin:
        return xmin, True

    return x, False


def clipped_mean(x: npt.NDArray[Any], p: int = 25) -> float:
    """Compute percentile-clipped mean."""
    mask1 = x >= np.percentile(x, p)
    mask2 = x <= np.percentile(x, 100 - p)
    mask = np.logical_and(mask1, mask2)
    return np.mean(x[mask])


def clipped_mean_rows(x: npt.NDArray[Any]) -> ArrF64:
    """Compute row-wise percentile-clipped mean."""
    return np.array([clipped_mean(xi) for xi in x])


def sparse2dense_coeffs(sparse_coefficients: list[float], powers: list[int]) -> list[float]:
    """Convert a list of sparse coefficients to a dense list."""
    degree = max(powers)

    # Fill missing powers with zeros
    coefficients = np.zeros(degree + 1)
    for i in range(degree + 1):
        if i in powers:
            coefficients[degree - i] = sparse_coefficients[powers.index(i)]
    return coefficients.tolist()
