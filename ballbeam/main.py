"""Main script for Open Ball & Beam.

This script calls the configurators as well as the Python interface.

Use this instead of running interface.py directly to ensure configurators have
been run & changed configuration parameters are updated.
"""

from pathlib import Path

from ballbeam.configurators.configurators import configure_calibrate_all
from ballbeam.static import COMMON_PATH

configure_calibrate_all()

# TODO(bgravell): eliminate use of exec() by refactoring interface.py to not use globals  # noqa: FIX002, TD003
with Path(COMMON_PATH.joinpath("interface.py")).open() as f:
    exec(f.read())  # noqa: S102
