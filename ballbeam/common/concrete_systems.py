"""Concrete systems."""

from ballbeam.common.hardware import HardwareSystem  # noqa: E402
from ballbeam.common.simulator import SimulatorSystem  # noqa: E402

# TODO(bgravell): Use a decorator to register systems with this registry   # noqa: TD003, FIX002
SYSTEM_CLASS_MAP = {
    "Simulator": SimulatorSystem,
    "Hardware": HardwareSystem,
}
