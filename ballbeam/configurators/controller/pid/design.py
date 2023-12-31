"""Design of PID controllers."""

from ballbeam.common.pickle_io import pickle_export
from ballbeam.static import CONFIGURATION_PATH

DIRECTORY_NAME_OUT = CONFIGURATION_PATH.joinpath("controller", "pid")


def make_controller_params() -> None:
    """Make the controller parameters."""
    controller_data = {
        "kp": 0.5,
        "ki": 0.5,
        "kd": 0.2,
        "error_mix": 0.5,
        "error_diff_mix": 0.2,
        "use_anti_windup": True,
    }
    pickle_export(dirname_out=DIRECTORY_NAME_OUT, filename_out="controller_params.pickle", data=controller_data)


def main() -> None:
    """Run the main function."""
    make_controller_params()


if __name__ == "__main__":
    main()
