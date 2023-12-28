import os


def create_directory(dirname_out) -> None:
    """Create target directory & all intermediate directories if nonexistent."""
    if not os.path.exists(dirname_out):
        os.makedirs(dirname_out)
        print("Directory '%s' created" % dirname_out)
