import os
from pathlib import Path

import ballbeam

# REPLACE THIS WITH THE PATH TO YOUR BALLBEAM REPO
BALLBEAM_PATH = Path("C:/Users/bjgra/GitHub/open-ball-beam/ballbeam")

COMMON_PATH = BALLBEAM_PATH.joinpath('common')

CONFIGURATORS_PATH = BALLBEAM_PATH.joinpath('configurators')
CALIBRATORS_PATH = BALLBEAM_PATH.joinpath('calibrators')
CONFIGURATION_PATH = BALLBEAM_PATH.joinpath('configuration')
CALIBRATION_PATH = BALLBEAM_PATH.joinpath('calibration')
