"""System representations of a ballbeam."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ballbeam.common.type_defs import ArrF64


class System(ABC):
    """A system representation of a ballbeam."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__()
        self.saturated = False
        self.ball_removed = False

    @abstractmethod
    def process(self, action: float) -> None:
        """Process an action."""

    @abstractmethod
    def observe(self) -> float:
        """Collect an observation."""

    @abstractmethod
    def reset(self, x: ArrF64 | None = None) -> None:
        """Reset the system."""

    @abstractmethod
    def shutdown(self) -> None:
        """Shut down the system."""


# Import these after System is defined to avoid circular imports.
from ballbeam.common.hardware import HardwareSystem  # noqa: E402
from ballbeam.common.simulator import SimulatorSystem  # noqa: E402

# TODO(bgravell): Use a decorator to register systems with this registry   # noqa: TD003, FIX002
SYSTEM_CLASS_MAP = {
    "Simulator": SimulatorSystem,
    "Hardware": HardwareSystem,
}
