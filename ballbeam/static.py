"""Static paths."""

from pathlib import Path

# REPLACE THIS WITH THE PATH TO YOUR ballbeam
BALLBEAM_PATH = Path("C:/Users/bjgra/GitHub/open-ball-beam/ballbeam")

COMMON_PATH = BALLBEAM_PATH.joinpath("common")

CONFIGURATORS_PATH = BALLBEAM_PATH.joinpath("configurators")
CALIBRATORS_PATH = BALLBEAM_PATH.joinpath("calibrators")
CONFIGURATION_PATH = BALLBEAM_PATH.joinpath("configuration")
CALIBRATION_PATH = BALLBEAM_PATH.joinpath("calibration")
