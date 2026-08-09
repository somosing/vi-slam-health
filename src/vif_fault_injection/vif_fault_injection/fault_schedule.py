from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FaultSchedule:
    """One-shot fault window expressed in seconds since node start."""

    enabled: bool
    start_after_sec: float
    duration_sec: float

    def active(self, elapsed_sec: float) -> bool:
        if not self.enabled or self.duration_sec <= 0.0:
            return False
        return self.start_after_sec <= elapsed_sec < (
            self.start_after_sec + self.duration_sec
        )
