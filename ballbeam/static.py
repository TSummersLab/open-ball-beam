"""Static paths."""

from pathlib import Path

import pygit2  # type: ignore[import]

ROOT_PATH = Path(pygit2.Repository(Path.cwd()).workdir)

BALLBEAM_PATH = ROOT_PATH.joinpath("ballbeam")

COMMON_PATH = BALLBEAM_PATH.joinpath("common")
CONFIGURATORS_PATH = BALLBEAM_PATH.joinpath("configurators")
CALIBRATORS_PATH = BALLBEAM_PATH.joinpath("calibrators")
CONFIGURATION_PATH = BALLBEAM_PATH.joinpath("configuration")
CALIBRATION_PATH = BALLBEAM_PATH.joinpath("calibration")
