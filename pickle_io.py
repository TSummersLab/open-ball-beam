"""Pickle import/export utilities"""
# Author: Ben Gravell

import pickle
import os


def create_directory(dirname_out):
    """Create target directory & all intermediate directories if nonexistent"""
    if not os.path.exists(dirname_out):
        os.makedirs(dirname_out)
        print("Directory '%s' created" % dirname_out)


def pickle_import(filename_in):
    pickle_off = open(filename_in, "rb")
    data = pickle.load(pickle_off)
    pickle_off.close()
    return data


def pickle_export(dirname_out, filename_out, data):
    create_directory(dirname_out)
    pickle_on = open(os.path.join(dirname_out, filename_out), "wb")
    pickle.dump(data, pickle_on)
    pickle_on.close()
