import numpy as np


def mix(a, b, x):
    return x * a + (1 - x) * b


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
    mask2 = x <= np.percentile(x, 100 - p)
    mask = np.logical_and(mask1, mask2)
    return np.mean(x[mask])


def clipped_mean_rows(x):
    return np.array([clipped_mean(xi) for xi in x])


def sparse2dense_coeffs(sparse_coefficients, powers):
    degree = max(powers)

    # Fill missing powers with zeros
    coefficients = np.zeros(degree + 1)
    for i in range(degree + 1):
        if i in powers:
            coefficients[degree - i] = sparse_coefficients[powers.index(i)]
    return coefficients.tolist()
