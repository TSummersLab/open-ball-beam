def print_arduino_vector(vector, var_name=None):
    if var_name is not None:
        print('%s = ' % var_name, end='')
    print('{' + ', '.join(['%.6f' % num for num in vector]) + '}')


class ConvertedObject(object):
    pass

def convert_dict_to_object(d):
    """Turns a (nested) dictionary into an object."""
    out = ConvertedObject()
    for key, value in d.items():
        attr = convert_dict_to_object(value) if isinstance(value, dict) else value
        setattr(out, key, attr)
    return out
