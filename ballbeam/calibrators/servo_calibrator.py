import os

import numpy as np
import matplotlib.pyplot as plt

from ballbeam.common.utility import print_arduino_vector
from ballbeam.common.extramath import clipped_mean_rows, sparse2dense_coeffs
from ballbeam.configurators.configurators import Configurator
from ballbeam.static import CALIBRATION_PATH


def get_servo_and_imu_data(filename):
    servo_outs = []
    raw_accels = []
    accel = []

    with open(filename) as file:
        collect_flag = False
        for line in file:
            if "BEGIN DATA COLLECTION" in line:
                collect_flag = True
                continue
            elif "END DATA COLLECTION" in line:
                collect_flag = False
                continue

            words = line.split(' ')
            first_word = words[0].strip()
            if collect_flag:
                if len(words) == 1:
                    if first_word.isnumeric():
                        servo_outs.append(int(first_word))
                    else:
                        raw_accels.append(accel)
                        accel = []
                elif len(words) == 3:
                    accel.append([float(word) for word in words])

    servo_outs = np.array(servo_outs)
    raw_accels = np.array(raw_accels)
    accels = np.array([clipped_mean_rows(raw_accel.T) for raw_accel in raw_accels])
    return servo_outs, accels


def make_servo_calibration_configurator(constants_configurator, hardware_configurator,
                                        show_print_arduino=False, show_plot=False):
    constants_config = constants_configurator.data_obj
    hardware_config = hardware_configurator.data_obj

    # Import servo calibration data
    servo_calibration_data_path = CALIBRATION_PATH.joinpath('servo_calibration_data.txt')
    servo_outs, accels = get_servo_and_imu_data(servo_calibration_data_path)
    mid_idx = np.where(servo_outs == hardware_config.SERVO.CMD.MID)[0][0]
    accel_offset = accels[mid_idx]
    delta_accels = accels - accel_offset
    delta_servo_outs = servo_outs - hardware_config.SERVO.CMD.MID

    # Angles are determined solely from the z-component of the accelerometer readings
    angles = -np.arcsin(delta_accels[:, 2]/constants_config.STD_GRAVITY_ACCEL)

    x = angles*hardware_config.BEAM.ANGLE_SCALE
    y = delta_servo_outs*hardware_config.SERVO.PWM_SCALE

    # Choose the polynomial powers to use in the least-squares regression
    powers = [5, 4, 3, 2, 1]

    # Form the data matrix
    X = np.vstack([x**power for power in powers]).T

    # Least-squares regression
    raw_coefficients = np.linalg.lstsq(X, y, rcond=None)[0]
    sparse_coefficients = raw_coefficients.tolist()
    coefficients = sparse2dense_coeffs(sparse_coefficients, powers)

    if show_print_arduino:
        # Print for Arduino
        print_arduino_vector(raw_coefficients, var_name='a2a_coeffs')

    if show_plot:
        # Plotting for sanity check
        poly = np.poly1d(coefficients)
        xmin = -8*constants_config.DEG2RAD*hardware_config.BEAM.ANGLE_SCALE
        xmax = 4*constants_config.DEG2RAD*hardware_config.BEAM.ANGLE_SCALE

        plt.figure()
        plt.scatter(x, y, c='b', label='true', zorder=10)
        t = np.linspace(xmin, xmax, 100)
        plt.plot(t, poly(t), lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
        plt.scatter(x, poly(x), s=50, lw=4, color='r', edgecolors='none', alpha=0.8, label='approx')
        plt.xlabel('Beam angle (rad) (scaled)')
        plt.ylabel('Servo pwm (microseconds) (scaled)')
        plt.legend()

        handles, labels = plt.gca().get_legend_handles_labels()
        order = [1, 2, 0]
        plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])

        plt.figure()
        plt.scatter(x, y - y, c='b', label='true', zorder=10)
        t = np.linspace(xmin, xmax, 100)
        if x[0] > x[1]:
            x, y = np.flip(x), np.flip(y)
        yt = np.interp(t, x, y)
        plt.plot(t, poly(t) - yt, lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
        plt.scatter(x, poly(x) - y, s=50, lw=4, edgecolors='none', color='r', alpha=0.8, label='approx')
        plt.xlabel('Beam angle (rad) (scaled)')
        plt.ylabel('Servo pwm error (microseconds) (scaled)')
        plt.legend()

        handles, labels = plt.gca().get_legend_handles_labels()
        order = [1, 2, 0]
        plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])

    name = 'servo_calibration'
    data = dict(coefficients=sparse_coefficients, powers=powers)
    description = "Servo calibration parameters"
    return Configurator(name, data, description)


if __name__ == '__main__':
    plt.close('all')

    from ballbeam.configurators.constants_configurator import make_constants_configurator
    from ballbeam.configurators.hardware_configurator import make_hardware_configurator
    constants_configurator = make_constants_configurator()
    hardware_configurator = make_hardware_configurator()

    make_servo_calibration_configurator(constants_configurator, hardware_configurator,
                                        show_print_arduino=True, show_plot=True)
