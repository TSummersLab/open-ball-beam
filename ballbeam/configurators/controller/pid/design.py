from ballbeam.common.pickle_io import pickle_export
from ballbeam.static import CONFIGURATION_PATH


# Pathing
dirname_out = CONFIGURATION_PATH.joinpath('controller', 'pid')


def make_controller_params():
    # Export controller parameters
    controller_data = dict(kp=0.5,
                           ki=0.1,
                           kd=0.25,
                           error_mix=0.50,
                           error_diff_mix=0.25,
                           anti_windup=True)
    pickle_export(dirname_out=dirname_out, filename_out='../../../configuration/controller/pid/controller_params.pickle', data=controller_data)


def main():
    make_controller_params()


if __name__ == "__main__":
    main()
