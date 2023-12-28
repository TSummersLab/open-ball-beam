import matplotlib.pyplot as plt
import numpy as np
import numpy.random as npr
from scipy.signal import convolve, windows

# Servo dynamics
npr.seed(1)

speed = 1
dt = 0.001

Ac = -speed
Bc = speed

A = 1 + Ac * dt
B = Bc * dt

x0 = 0.0

tt = 100
T = int(tt / dt)
t_hist = np.arange(T) * dt
x_hist = np.zeros(T + 1)
u_hist = convolve(npr.randn(T), windows.hann(T // 5), mode="same")

x = np.copy(x0)
x_hist[0] = x0

for t in range(T):
    u = u_hist[t]
    x = A * x + B * u
    x_hist[t + 1] = x

plt.plot(t_hist, x_hist[:-1], t_hist, u_hist)
