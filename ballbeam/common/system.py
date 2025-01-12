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
