"""Utility functions."""


from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy.typing as npt


def print_arduino_vector(vector: npt.NDArray[Any], var_name: str | None = None) -> None:
    if var_name is not None:
        print(f"{var_name} = ", end="")
    print("{" + ", ".join([f"{num:.6f}" for num in vector]) + "}")


class ConvertedObject:
    pass


def convert_dict_to_object(d: dict[str, Any]) -> ConvertedObject:
    """Turns a (nested) dictionary into an object."""
    out = ConvertedObject()
    for key, value in d.items():
        attr = convert_dict_to_object(value) if isinstance(value, dict) else value
        setattr(out, key, attr)
    return out
