"""Main script for Open Ball & Beam."""

import argparse

from ballbeam.common.interface import Interface
from ballbeam.configurators.configs import BallBeamConfig
from ballbeam.configurators.configurators import configure_calibrate_all


def _get_parsed_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--configure",
        action="store_true",
        help="Run all configurators and calibrators, updating parameters.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _get_parsed_args()
    if args.configure:
        configure_calibrate_all()
    config = BallBeamConfig.from_filesystem()
    Interface(config.interface).run()
