from dataclasses import dataclass
from typing import Protocol

from robot_pickup.models import Detection, DistanceReading, Scene


class TargetRanger(Protocol):
    @property
    def aim_x(self) -> float:
        """Horizontal point in the frame to align the robot with."""

    def estimate(self, target: Detection, scene: Scene, reading: DistanceReading) -> float | None:
        """Return distance only when the measurement plausibly belongs to this target."""


@dataclass(frozen=True)
class BeamSettings:
    calibrated: bool = False
    center_x: float = 0.0
    center_y: float = 0.0
    half_width: float = 0.12
    half_height: float = 0.20
    max_skew_s: float = 0.18

    def __post_init__(self) -> None:
        if type(self.calibrated) is not bool or not (
            0 < self.half_width <= 1 and 0 < self.half_height <= 1
            and abs(self.center_x) + self.half_width <= 1
            and abs(self.center_y) + self.half_height <= 1 and 0 < self.max_skew_s
        ):
            raise ValueError("the beam must fit within the image and have positive time tolerance")


class BeamAssociation:
    def __init__(self, settings: BeamSettings) -> None:
        self.settings = settings

    @property
    def aim_x(self) -> float:
        return self.settings.center_x

    def estimate(self, target: Detection, scene: Scene, reading: DistanceReading) -> float | None:
        beam = self.settings
        if not beam.calibrated or not reading.valid or target not in scene.objects:
            return None
        if abs(scene.observed_at - reading.sample.measured_at) > beam.max_skew_s:
            return None
        if abs(target.cx - beam.center_x) > beam.half_width or abs(target.cy - beam.center_y) > beam.half_height:
            return None
        cx = (beam.center_x + 1) * scene.width / 2
        cy = (beam.center_y + 1) * scene.height / 2
        box = target.bbox
        if not (box.x1 <= cx < box.x2 and box.y1 <= cy < box.y2):
            return None
        dx = beam.half_width * scene.width / 2
        dy = beam.half_height * scene.height / 2
        for other in scene.objects:
            if other is target:
                continue
            box = other.bbox
            if box.x1 <= cx + dx and box.x2 >= cx - dx and box.y1 <= cy + dy and box.y2 >= cy - dy:
                return None
        return reading.filtered_cm
