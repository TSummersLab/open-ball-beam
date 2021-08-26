def print_arduino_vector(vector, var_name=None):
    if var_name is not None:
        print('%s = ' % var_name, end='')
    print('{' + ', '.join(['%.6f' % num for num in vector]) + '}')
