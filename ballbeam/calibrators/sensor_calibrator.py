"""Sensor calibration."""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np

from ballbeam.common.extramath import clipped_mean_rows, sparse2dense_coeffs
from ballbeam.common.utility import print_arduino_vector
from ballbeam.configurators.configurators import Configurator
from ballbeam.static import CALIBRATION_PATH

if TYPE_CHECKING:
    from ballbeam.common.types import NDA


def get_distance_and_sensor_data(filename: Path | str) -> tuple[NDA, NDA, NDA]:
    """Get distance and sensor data from a file."""
    distances: list[int] = []
    raw_readings: list[list[float]] = []
    reading: list[float] = []

    path = Path(filename)
    with path.open("r") as file:
        collect_flag = False
        for line in file:
            if "BEGIN DATA COLLECTION" in line:
                collect_flag = True
                continue
            if "END DATA COLLECTION" in line:
                collect_flag = False
                continue

            words = line.split(" ")
            first_word = words[0].strip()
            last_word = words[-1].strip()
            if collect_flag:
                if first_word == "":
                    raw_readings.append(reading)
                    reading = []
                    continue
                if "distance" in words:
                    distances.append(int(last_word))
                else:
                    reading.append(float(first_word))

    distances_arr = np.array(distances)
    raw_readings_arr = np.array(raw_readings)
    readings_arr = clipped_mean_rows(raw_readings_arr)
    return distances_arr, readings_arr, raw_readings_arr


def make_sensor_calibration_configurator(
    hardware_configurator: Configurator,
    *,
    show_print_arduino: bool = False,
    show_plot: bool = False,
) -> Configurator:
    """Make a sensor calibration configurator."""
    hardware_config = hardware_configurator.data_obj

    # Import sensor calibration data
    sensor_calibration_data_path = CALIBRATION_PATH.joinpath("sensor_calibration_data.txt")
    distances, readings, raw_readings = get_distance_and_sensor_data(sensor_calibration_data_path)
    mid_idx = np.where(distances == hardware_config.SENSOR.DISTANCE.MID)[0][0]  # type: ignore[attr-defined]
    READING_OFFSET = float(readings[mid_idx])
    delta_readings = readings - READING_OFFSET
    delta_raw_readings = raw_readings - READING_OFFSET
    delta_distances = distances - hardware_config.SENSOR.DISTANCE.MID  # type: ignore[attr-defined]

    x = delta_readings * hardware_config.SENSOR.READING_SCALE  # type: ignore[attr-defined]
    xs = delta_raw_readings * hardware_config.SENSOR.READING_SCALE  # type: ignore[attr-defined]
    y = delta_distances * hardware_config.SENSOR.OBSERVATION_SCALE  # type: ignore[attr-defined]

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
        print_arduino_vector(raw_coefficients, var_name="R2O_COEFFS[]")

    if show_plot:
        # Plotting for sanity check
        poly = np.poly1d(coefficients)
        xmin, xmax = -150 * hardware_config.SENSOR.READING_SCALE, 150 * hardware_config.SENSOR.READING_SCALE  # type: ignore[attr-defined]

        # Curve plot
        plt.figure()
        plt.scatter(x, y, c="b", label="true", zorder=10)
        t = np.linspace(xmin, xmax, 100)
        plt.plot(t, poly(t), lw=2, linestyle="--", color="r", alpha=0.5, label="approx", zorder=1)
        plt.scatter(x, poly(x), s=50, lw=4, color="r", edgecolors="none", alpha=0.8, label="approx")
        for i in range(x.size):
            plt.scatter(xs[i], np.repeat(y[i], xs[i].size), c="g", edgecolors="none", alpha=0.4, label="samples")

        plt.xlabel("Delta reading (mm) (scaled)")
        plt.ylabel("Delta distance (mm) (scaled)")
        plt.legend()

        handles, labels = plt.gca().get_legend_handles_labels()  # type: ignore[no-untyped-call]
        order = [1, 2, 0]
        plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])

        # Error plot
        plt.figure()
        plt.scatter(x, y - y, c="b", label="true", zorder=10)
        t = np.linspace(xmin, xmax, 100)

        yt = np.interp(t, x, y)
        plt.plot(t, poly(t) - yt, lw=2, linestyle="--", color="r", alpha=0.5, label="approx", zorder=1)
        plt.scatter(x, poly(x) - y, s=50, lw=4, edgecolors="none", color="r", alpha=0.8, label="approx")
        plt.xlabel("Delta reading (mm) (scaled)")
        plt.ylabel("Delta distance error (mm) (scaled)")
        plt.legend()

        handles, labels = plt.gca().get_legend_handles_labels()  # type: ignore[no-untyped-call]
        order = [1, 2, 0]
        plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])
        plt.show()  # type: ignore[no-untyped-call]

    name = "sensor_calibration"
    data = {"coefficients": sparse_coefficients, "powers": powers, "READING_OFFSET": READING_OFFSET}
    description = "Servo calibration parameters"
    return Configurator(name, data, description)


if __name__ == "__main__":
    from ballbeam.configurators.hardware_configurator import make_hardware_configurator

    hardware_configurator = make_hardware_configurator()

    make_sensor_calibration_configurator(
        hardware_configurator,
        show_print_arduino=True,
        show_plot=True,
    )
