"""Shared data models independent of OpenCV and the Arduino protocol."""

from dataclasses import dataclass
from math import isfinite
from typing import TypeGuard


@dataclass(frozen=True)
class BoundingBox:
    """Half-open pixel coordinates (left, top, right, bottom)."""

    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def area(self) -> int:
        return (self.x2 - self.x1) * (self.y2 - self.y1)


@dataclass(frozen=True)
class Detection:
    label: str
    bbox: BoundingBox
    cx: float
    cy: float
    area_fraction: float
    confidence: float | None = None
    object_id: int | None = None

    @classmethod
    def from_box(
        cls, label: str, box: BoundingBox, width: int, height: int,
        confidence: float | None = None,
    ) -> "Detection":
        return cls(
            label, box,
            (box.x1 + box.x2) / width - 1,
            (box.y1 + box.y2) / height - 1,
            box.area / (width * height), confidence,
        )


@dataclass(frozen=True)
class Scene:
    """Only detections visible in this frame. Timestamps use host monotonic seconds."""

    observed_at: float
    width: int
    height: int
    objects: tuple[Detection, ...]


@dataclass(frozen=True)
class DistanceSample:
    """One measurement of the whole beam, timestamped in host monotonic seconds."""

    cm: float | None
    measured_at: float
    valid: bool = True
    reason: str = ""


@dataclass(frozen=True)
class DistanceReading:
    sample: DistanceSample | None = None
    filtered_cm: float | None = None
    reason: str = "no distance data"

    @property
    def valid(self) -> bool:
        return self.sample is not None and self.sample.valid and self.filtered_cm is not None

    @property
    def raw_cm(self) -> float | None:
        return self.sample.cm if self.valid and self.sample is not None else None


def finite_number(value: object) -> TypeGuard[int | float]:
    """Check numeric telemetry and reject JSON booleans, NaN and overflow."""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and -float("inf") < value < float("inf")


def is_fresh(timestamp: float, now: float, max_age_s: float) -> bool:
    return isfinite(timestamp) and 0 <= now - timestamp <= max_age_s
