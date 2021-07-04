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
