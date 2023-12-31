"""Pickle import and export utilities."""
from __future__ import annotations

import pickle
from pathlib import Path
from typing import Any

from ballbeam.common.path_io import create_directory


def pickle_import(path: Path | str) -> Any:
    """Import data from a pickle file.

    Args:
    ----
    path: The path to the file to import from.

    Returns:
    -------
    Any: The data loaded from the pickle file.
    """
    path = Path(path)
    with path.open("rb") as file:
        return pickle.load(file)


def pickle_export(dirname_out: Path | str, filename_out: str, data: Any) -> None:
    """Export data to a pickle file.

    Args:
    ----
    dirname_out: The directory where the file will be saved.
    filename_out: The name of the file to save the data in.
    data: The data to be saved.
    """
    create_directory(dirname_out)
    path = Path(dirname_out) / filename_out
    with path.open("wb") as file:
        pickle.dump(data, file)
