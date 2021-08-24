import numpy as np
import matplotlib.pyplot as plt

from ballbeam.common.extramath import clipped_mean_rows
from ballbeam.common.pickle_io import pickle_export
from ballbeam.common.settings import DISTANCE_MID


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


# Import servo calibration data
distances, readings, raw_readings = get_distance_and_sensor_data('sensor_calibration_data.txt')
delta_distances = distances - DISTANCE_MID

x = readings
y = delta_distances

# Choose the polynomial powers to use in the least-squares regression
powers = [3, 2, 1, 0]
degree = max(powers)

# Form the data matrix
X = np.vstack([x**power for power in powers]).T

# Least-squares regression
raw_coefficients = np.linalg.lstsq(X, y, rcond=None)[0]

# Fill missing powers with zeros
coefficients = np.zeros(degree+1)
for i in range(degree+1):
    if i in powers:
        coefficients[degree-i] = raw_coefficients[powers.index(i)]

# Export the coefficients
pickle_export(dirname_out='.', filename_out='sensor_calibration_coefficients.pkl', data=coefficients)


# plotting for sanity check
poly = np.poly1d(coefficients)
reading_min, reading_max = 0, 300

plt.close('all')
plt.figure()
plt.scatter(readings, delta_distances, c='b', label='true', zorder=10)
t = np.linspace(reading_min, reading_max, 100)
plt.plot(t, poly(t), lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
plt.scatter(readings, poly(readings), s=50, lw=4, color='r', edgecolors='none', alpha=0.8, label='approx')
plt.xlabel('Reading (mm)')
plt.ylabel('Delta distance (mm)')
plt.legend()

handles, labels = plt.gca().get_legend_handles_labels()
order = [1, 2, 0]
plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])


plt.figure()
plt.scatter(readings, delta_distances - delta_distances, c='b', label='true', zorder=10)
t = np.linspace(reading_min, reading_max, 100)

yt = np.interp(t, readings, delta_distances)
plt.plot(t, poly(t)-yt, lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
plt.scatter(readings, poly(readings) - delta_distances, s=50, lw=4, edgecolors='none', color='r', alpha=0.8, label='approx')
plt.xlabel('Reading (mm)')
plt.ylabel('Delta distance error (mm)')
plt.legend()

handles, labels = plt.gca().get_legend_handles_labels()
order = [1, 2, 0]
plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])
