import os
from pathlib import Path

import ballbeam

BALLBEAM_PATH = Path(ballbeam.__file__).parent
COMMON_PATH = BALLBEAM_PATH.joinpath('common')

CONFIGURATORS_PATH = BALLBEAM_PATH.joinpath('configurators')
CALIBRATORS_PATH = BALLBEAM_PATH.joinpath('calibrators')
CONFIGURATION_PATH = BALLBEAM_PATH.joinpath('configuration')
CALIBRATION_PATH = BALLBEAM_PATH.joinpath('calibration')
