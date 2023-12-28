# This is a script that calls the configurators as well as the Python interface. Use this instead of running
# interface.py directly to ensure configurators have been run & changed configuration parameters are updated.

from ballbeam.static import COMMON_PATH, CONFIGURATORS_PATH

exec(open(CONFIGURATORS_PATH.joinpath("configurators.py")).read())
exec(open(COMMON_PATH.joinpath("interface.py")).read())
