"""Utility functions."""
from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ballbeam.common.type_defs import ArrF64


def print_arduino_vector(vector: ArrF64, var_name: str | None = None) -> None:
    """Print a vector as a string for use in Arduino code."""
    if var_name is not None:
        print(f"{var_name} = ", end="")
    print("{" + ", ".join([f"{num:.6f}" for num in vector]) + "}")


class ConvertedObject:
    """Dummy object for conversion from dict."""


def convert_dict_to_object(d: dict[str, Any]) -> ConvertedObject:
    """Convert a (nested) dictionary to an object."""
    out = ConvertedObject()
    for key, value in d.items():
        attr = convert_dict_to_object(value) if isinstance(value, dict) else value
        setattr(out, key, attr)
    return out


def instantiate_object_by_class_name(class_name: str, class_map: dict[str, Any]) -> Any:
    """Instantiate an object by class name using a class map."""
    selected_class = class_map.get(class_name)
    if selected_class is None:
        msg = f'Failed to find class for class name "{class_name}", must be one of {sorted(class_map)}.'
        raise ValueError(msg)
    return selected_class()
