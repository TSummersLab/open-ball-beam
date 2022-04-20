def print_arduino_vector(vector, var_name=None):
    if var_name is not None:
        print('%s = ' % var_name, end='')
    print('{' + ', '.join(['%.6f' % num for num in vector]) + '}')


class Dict2Obj(object):
    """Turns a (nested) dictionary into an object."""
    def __init__(self, dictionary):
        for key in dictionary:
            val = dictionary[key]
            attr = Dict2Obj(val) if isinstance(val, dict) else val
            setattr(self, key, attr)
