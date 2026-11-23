import json
from dataclasses import dataclass, field
from math import isfinite
from pathlib import Path

from robot_pickup.communication.commands import CommandKind, LinkSettings
from robot_pickup.control.controller import ControllerSettings, TargetPolicy
from robot_pickup.perception.camera import CameraSettings
from robot_pickup.perception.detector import DEFAULT_COLORS, ColorClass, HSVDetector, HSVRange, YOLODetector
from robot_pickup.perception.tracking import CentroidTracker, VisionPipeline
from robot_pickup.sensors.association import BeamSettings
from robot_pickup.sensors.distance import DistanceSettings


@dataclass(frozen=True)
class PerceptionSettings:
    backend: str = "hsv"
    colors: tuple[ColorClass, ...] = DEFAULT_COLORS
    min_area_px: int = 100
    max_area_fraction: float = 0.55
    morphology_size: int = 3
    max_displacement: float = 0.22
    track_gap_s: float = 0.70
    yolo_weights: str = "best.pt"
    yolo_confidence: float = 0.5

    def __post_init__(self) -> None:
        if self.backend not in ("hsv", "yolo") or not self.colors:
            raise ValueError("use the hsv or yolo backend and configure at least one HSV color")
        if not isinstance(self.yolo_weights, str) or not self.yolo_weights or not 0 <= self.yolo_confidence <= 1:
            raise ValueError("invalid YOLO weights path or confidence threshold")

    def pipeline(self) -> VisionPipeline:
        if self.backend == "yolo":
            detector = YOLODetector(self.yolo_weights, self.yolo_confidence)
        else:
            detector = HSVDetector(self.colors, self.min_area_px, self.max_area_fraction, self.morphology_size)
        return VisionPipeline(detector, CentroidTracker(self.max_displacement, self.track_gap_s))


@dataclass(frozen=True)
class SerialSettings:
    baudrate: int = 115200
    write_timeout_s: float = 0.05
    startup_s: float = 2.0
    combined_velocity: bool = False

    def __post_init__(self) -> None:
        if type(self.baudrate) is not int or self.baudrate <= 0:
            raise ValueError("serial baudrate must be a positive integer")
        if not (0 < self.write_timeout_s <= 0.2 and 0 <= self.startup_s) or type(self.combined_velocity) is not bool:
            raise ValueError("invalid serial startup, write timeout, or combined_velocity setting")


@dataclass(frozen=True)
class RuntimeSettings:
    tick_hz: float = 30.0
    http_poll_s: float = 0.10
    http_timeout_s: float = 0.25

    def __post_init__(self) -> None:
        if not (1 <= self.tick_hz <= 120 and 0 < self.http_poll_s and 0 < self.http_timeout_s):
            raise ValueError("invalid tick rate or HTTP timing")


@dataclass(frozen=True)
class Config:
    perception: PerceptionSettings = field(default_factory=PerceptionSettings)
    camera: CameraSettings = field(default_factory=CameraSettings)
    target: TargetPolicy = field(default_factory=TargetPolicy)
    distance: DistanceSettings = field(default_factory=DistanceSettings)
    beam: BeamSettings = field(default_factory=BeamSettings)
    control: ControllerSettings = field(default_factory=ControllerSettings)
    link: LinkSettings = field(default_factory=LinkSettings)
    serial: SerialSettings = field(default_factory=SerialSettings)
    runtime: RuntimeSettings = field(default_factory=RuntimeSettings)

    def __post_init__(self) -> None:
        if not self.distance.min_cm <= self.control.pickup_floor_cm < self.control.pickup_cm <= self.distance.max_cm:
            raise ValueError("the pickup window must fit within the sensor's range")
        if 1 / self.runtime.tick_hz >= self.link.heartbeat_s:
            raise ValueError("the control tick interval must be shorter than the heartbeat interval")


def _reject_constant(value: str) -> None:
    raise ValueError(f"invalid JSON constant {value}")


def _parse_float(value: str) -> float:
    number = float(value)
    if not isfinite(number):
        raise ValueError("configuration numbers must be finite")
    return number


def load_config(path: Path | None) -> Config:
    if path is None:
        return Config()
    with path.open(encoding="utf-8-sig") as file:
        data = json.load(file, parse_constant=_reject_constant, parse_float=_parse_float)
    if not isinstance(data, dict) or data.keys() - Config.__dataclass_fields__.keys():
        raise ValueError("unknown or invalid configuration section")
    if any(not isinstance(section, dict) for section in data.values()):
        raise ValueError("configuration sections must be JSON objects")

    perception = data.get("perception", {})
    if "colors" in perception:
        colors = perception["colors"]
        if not isinstance(colors, dict) or any(not isinstance(ranges, list) for ranges in colors.values()):
            raise ValueError("colors must map labels to lists of HSV ranges")
        perception["colors"] = tuple(
            ColorClass(label, tuple(HSVRange(tuple(bounds["low"]), tuple(bounds["high"])) for bounds in ranges))
            for label, ranges in colors.items()
        )
    target = data.get("target", {})
    if "labels" in target:
        labels = target["labels"]
        if not isinstance(labels, list) or any(not isinstance(label, str) or not label for label in labels):
            raise ValueError("target labels must be a list of nonempty strings")
        target["labels"] = tuple(labels)
    control = data.get("control", {})
    if "pickup_sequence" in control:
        sequence = control["pickup_sequence"]
        if not isinstance(sequence, list):
            raise ValueError("pickup_sequence must be a list of arm commands")
        control["pickup_sequence"] = tuple(CommandKind(command) for command in sequence)

    return Config(
        perception=PerceptionSettings(**perception), camera=CameraSettings(**data.get("camera", {})),
        target=TargetPolicy(**target), distance=DistanceSettings(**data.get("distance", {})),
        beam=BeamSettings(**data.get("beam", {})), control=ControllerSettings(**control),
        link=LinkSettings(**data.get("link", {})), serial=SerialSettings(**data.get("serial", {})),
        runtime=RuntimeSettings(**data.get("runtime", {})),
    )
