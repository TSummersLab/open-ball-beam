"""Data logging."""

from __future__ import annotations

import dataclasses
import json
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


@dataclasses.dataclass
class LogData:
    """Log data record."""

    state_estimate: list[float]
    observation: float
    action: float
    setpoint: float
    cost_dict: dict[str, float]
    time_since_last: float
    time_now: float
    time_step: int


class Logger(ABC):
    """Data logger."""

    @abstractmethod
    def append(self, log_data: LogData) -> None:
        """Append a record."""

    @abstractmethod
    def dump(self, path: Path) -> None:
        """Dump records."""


class NoneLogger(Logger):
    """Logger that does not log any data."""

    def append(self, log_data: LogData) -> None:
        """Append a record."""

    def dump(self, path: Path) -> None:
        """Dump records."""


class FullLogger(Logger):
    """Logger that logs the full history of data."""

    def __init__(self) -> None:
        """Initialize."""
        self.log_datas: list[LogData] = []

    def append(self, log_data: LogData) -> None:
        """Append a record."""
        self.log_datas.append(log_data)

    def dump(self, path: Path) -> None:
        """Dump records."""
        json_data = [dataclasses.asdict(log_data) for log_data in self.log_datas]
        with path.open("w") as f:
            json.dump(json_data, f, indent=4)
        print(f"Wrote log data to {path}")


# Register all classes with this map
# TODO(bgravell): Refactor to decorate each concrete class with this registration # noqa: TD003, FIX002
LOGGER_CLASS_MAP = {
    "None": NoneLogger,
    "Full": FullLogger,
}
