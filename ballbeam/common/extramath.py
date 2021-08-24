import numpy as np


def mix(a, b, x):
    return x*a + (1 - x)*b


def saturate(x, xmin, xmax):
    # similar to np.clip, but in addition returns a bool that indicates whether clipping occurred
    if x > xmax:
        return xmax, True
    elif x < xmin:
        return xmin, True
    else:
        return x, False


def clipped_mean(x, p=25):
    mask1 = x >= np.percentile(x, p)
    mask2 = x <= np.percentile(x, 100-p)
    mask = np.logical_and(mask1, mask2)
    return np.mean(x[mask])


def clipped_mean_rows(x):
    return np.array([clipped_mean(xi) for xi in x])
