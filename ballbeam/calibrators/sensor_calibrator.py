import os

import numpy as np
import matplotlib.pyplot as plt

from ballbeam.common.utility import print_arduino_vector
from ballbeam.common.extramath import clipped_mean_rows, sparse2dense_coeffs
from ballbeam.configurators.configurators import Configurator
from ballbeam.static import CALIBRATION_PATH


def get_distance_and_sensor_data(filename):
    distances = []
    raw_readings = []
    reading = []

    with open(filename) as file:
        collect_flag = False
        for line in file:
            # print(line, end='')

            if "BEGIN DATA COLLECTION" in line:
                collect_flag = True
                continue
            elif "END DATA COLLECTION" in line:
                collect_flag = False
                continue

            words = line.split(' ')
            first_word = words[0].strip()
            last_word = words[-1].strip()
            if collect_flag:
                if first_word == '':
                    raw_readings.append(reading)
                    reading = []
                    continue
                if 'distance' in words:
                    distances.append(int(last_word))
                else:
                    reading.append(float(first_word))

    distances = np.array(distances)
    raw_readings = np.array(raw_readings)
    readings = clipped_mean_rows(raw_readings)
    return distances, readings, raw_readings


def make_sensor_calibration_configurator(constants_configurator, hardware_configurator,
                                         show_print_arduino=False, show_plot=False):

    constants_config = constants_configurator.data_obj
    hardware_config = hardware_configurator.data_obj

    # Import sensor calibration data
    sensor_calibration_data_path = CALIBRATION_PATH.joinpath('sensor_calibration_data.txt')
    distances, readings, raw_readings = get_distance_and_sensor_data(sensor_calibration_data_path)
    mid_idx = np.where(distances == hardware_config.SENSOR.DISTANCE.MID)[0][0]
    READING_OFFSET = float(readings[mid_idx])
    delta_readings = readings - READING_OFFSET
    delta_raw_readings = raw_readings - READING_OFFSET
    delta_distances = distances - hardware_config.SENSOR.DISTANCE.MID

    x = delta_readings*hardware_config.SENSOR.READING_SCALE
    xs = delta_raw_readings*hardware_config.SENSOR.READING_SCALE
    y = delta_distances*hardware_config.SENSOR.OBSERVATION_SCALE

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
        print_arduino_vector(raw_coefficients, var_name='r2o_coeffs')

    if show_plot:
        # Plotting for sanity check
        poly = np.poly1d(coefficients)
        xmin, xmax = -150*hardware_config.SENSOR.READING_SCALE, 150*hardware_config.SENSOR.READING_SCALE

        # Curve plot
        plt.figure()
        plt.scatter(x, y, c='b', label='true', zorder=10)
        t = np.linspace(xmin, xmax, 100)
        plt.plot(t, poly(t), lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
        plt.scatter(x, poly(x), s=50, lw=4, color='r', edgecolors='none', alpha=0.8, label='approx')
        for i in range(x.size):
            plt.scatter(xs[i], np.repeat(y[i], xs[i].size), c='g', edgecolors='none', alpha=0.4, label='samples')

        plt.xlabel('Delta reading (mm) (scaled)')
        plt.ylabel('Delta distance (mm) (scaled)')
        plt.legend()

        handles, labels = plt.gca().get_legend_handles_labels()
        order = [1, 2, 0]
        plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])

        # Error plot
        plt.figure()
        plt.scatter(x, y - y, c='b', label='true', zorder=10)
        t = np.linspace(xmin, xmax, 100)

        yt = np.interp(t, x, y)
        plt.plot(t, poly(t) - yt, lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
        plt.scatter(x, poly(x) - y, s=50, lw=4, edgecolors='none', color='r', alpha=0.8, label='approx')
        plt.xlabel('Delta reading (mm) (scaled)')
        plt.ylabel('Delta distance error (mm) (scaled)')
        plt.legend()

        handles, labels = plt.gca().get_legend_handles_labels()
        order = [1, 2, 0]
        plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])
    plt.show()

    name = 'sensor_calibration'
    data = dict(coefficients=sparse_coefficients, powers=powers, READING_OFFSET=READING_OFFSET)
    description = "Servo calibration parameters"
    return Configurator(name, data, description)


if __name__ == '__main__':
    plt.close('all')

    from ballbeam.configurators.constants_configurator import make_constants_configurator
    from ballbeam.configurators.hardware_configurator import make_hardware_configurator
    constants_configurator = make_constants_configurator()
    hardware_configurator = make_hardware_configurator()

    make_sensor_calibration_configurator(constants_configurator, hardware_configurator,
                                         show_print_arduino=True, show_plot=True)
