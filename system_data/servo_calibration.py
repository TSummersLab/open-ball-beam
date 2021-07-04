import numpy as np
import matplotlib.pyplot as plt
from pickle_io import pickle_export
from settings import RAD2DEG, SERVO_CMD_MID

# Import calibration data
filename = 'servo_calibration_data.txt'

servo_outs = []
raw_accels = []
accel = []

with open(filename) as file:
    for line in file:
        # print(line, end='')
        words = line.split(' ')
        if len(words) == 1:
            word = words[0].split('\n')[0]
            if word.isnumeric():
                servo_outs.append(int(word))
            else:
                raw_accels.append(accel)
                accel = []
        elif len(words) == 3:
            accel.append([float(word) for word in words])

servo_outs = np.array(servo_outs)
raw_accels = np.array(raw_accels)
accels = np.mean(raw_accels, axis=1)

delta_accels = accels
delta_servo_outs = servo_outs - SERVO_CMD_MID
standard_gravity_accel = 9.80665

angles = np.arcsin(delta_accels[:, 2]/standard_gravity_accel)*RAD2DEG

x = angles
y = delta_servo_outs

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
pickle_export(dirname_out='.', filename_out='servo_calibration_coefficients.pkl', data=coefficients)


# plotting for sanity check
poly = np.poly1d(coefficients)

plt.close('all')
plt.figure()
plt.scatter(angles, delta_servo_outs, c='b', label='true', zorder=10)
t = np.linspace(-5, 5, 100)
plt.plot(t, poly(t), lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
plt.scatter(angles, poly(angles), s=50, lw=4, color='r', edgecolors='none', alpha=0.8, label='approx')
plt.xlabel('Beam angle (degrees)')
plt.ylabel('Servo pwm (microseconds)')
plt.legend()

handles, labels = plt.gca().get_legend_handles_labels()
order = [1, 2, 0]
plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])


plt.figure()
plt.scatter(angles, delta_servo_outs-delta_servo_outs, c='b', label='true', zorder=10)
t = np.linspace(-5, 5, 100)
if angles[0] > angles[1]:
    angles, delta_servo_outs = np.flip(angles), np.flip(delta_servo_outs)
yt = np.interp(t, angles, delta_servo_outs)
plt.plot(t, poly(t)-yt, lw=2, linestyle='--', color='r', alpha=0.5, label='approx', zorder=1)
plt.scatter(angles, poly(angles)-delta_servo_outs, s=50,  lw=4, edgecolors='none', color='r', alpha=0.8, label='approx')
plt.xlabel('Beam angle (degrees)')
plt.ylabel('Servo pwm error (microseconds)')
plt.legend()

handles, labels = plt.gca().get_legend_handles_labels()
order = [1, 2, 0]
plt.legend([handles[idx] for idx in order], [labels[idx] for idx in order])
