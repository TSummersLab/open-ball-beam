import numpy as np
import matplotlib.pyplot as plt

from ballbeam.common.utility import print_arduino_vector
from ballbeam.common.extramath import clipped_mean_rows
from ballbeam.common.pickle_io import pickle_export
from ballbeam.common.settings import RAD2DEG, DEG2RAD, SERVO_CMD_MID, STD_GRAVITY_ACCEL, PWM_SCALE, BEAM_ANGLE_SCALE


def get_servo_and_imu_data(filename):
    servo_outs = []
    raw_accels = []
    accel = []

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
    # accels = np.mean(raw_accels, axis=1)
    accels = np.array([clipped_mean_rows(raw_accel.T) for raw_accel in raw_accels])
    return servo_outs, accels


# Import servo calibration data
servo_outs, accels = get_servo_and_imu_data('servo_calibration_data.txt')
mid_idx = np.where(servo_outs == SERVO_CMD_MID)[0][0]
accel_offset = accels[mid_idx]
delta_accels = accels - accel_offset
delta_servo_outs = servo_outs - SERVO_CMD_MID

# Angles are determined solely from the z-component of the accelerometer readings
angles = -np.arcsin(delta_accels[:, 2]/STD_GRAVITY_ACCEL)

x = angles*BEAM_ANGLE_SCALE
y = delta_servo_outs*PWM_SCALE

# Choose the polynomial powers to use in the least-squares regression
powers = [5, 4, 3, 2, 1]
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
pickle_export(dirname_out='.', filename_out='servo_calibration_coefficients.pkl', data=coefficients)

# Print for Arduino
print_arduino_vector(raw_coefficients, var_name='a2a_coeffs')

# plotting for sanity check
poly = np.poly1d(coefficients)
xmin, xmax = -8*DEG2RAD*BEAM_ANGLE_SCALE, 4*DEG2RAD*BEAM_ANGLE_SCALE

plt.close('all')
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
plt.scatter(x, y-y, c='b', label='true', zorder=10)
t = np.linspace(xmin, xmax, 100)
if x[0] > x[1]:
    x, y = np.flip(x), np.flip(y)
yt = np.interp(t, x, y)
plt.plot(t, poly(t)-yt, lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
plt.scatter(x, poly(x)-y, s=50,  lw=4, edgecolors='none', color='r', alpha=0.8, label='approx')
plt.xlabel('Beam angle (rad) (scaled)')
plt.ylabel('Servo pwm error (microseconds) (scaled)')
plt.legend()

handles, labels = plt.gca().get_legend_handles_labels()
order = [1, 2, 0]
plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])
