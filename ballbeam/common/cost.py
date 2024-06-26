"""Classes representing costs."""

from __future__ import annotations

import copy


class Cost:
    """Base class for a cost."""

    def __init__(self, e0: float = 0.0, u0: float = 0.0) -> None:
        """Initialize."""
        self.e = copy.copy(e0)  # output error
        self.u = copy.copy(u0)  # action

    def take(self, e: float, u: float) -> dict[str, float]:
        """Take the cost of the current error and action."""
        c_error = 10 * abs(e)
        c_action = 8 * abs(u)
        c_action_diff = 3 * abs(u - self.u)
        c = c_error + c_action + c_action_diff
        self.e = e
        self.u = u
        return {"c": c, "c_error": c_error, "c_action": c_action, "c_action_diff": c_action_diff}


# Register all classes with this map
# TODO(bgravell): Refactor to decorate each concrete class with this registration # noqa: TD003, FIX002
COST_CLASS_MAP = {"Default": Cost}
