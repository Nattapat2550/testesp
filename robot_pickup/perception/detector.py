from dataclasses import dataclass
from math import ceil, floor
from typing import Protocol

import cv2
import numpy as np
from numpy.typing import NDArray

from robot_pickup.models import BoundingBox, Detection

Frame = NDArray[np.uint8]


class Detector(Protocol):
    def detect(self, frame: Frame) -> list[Detection]:
        """Detect objects in a BGR frame without assigning tracking IDs."""


@dataclass(frozen=True)
class HSVRange:
    low: tuple[int, int, int]
    high: tuple[int, int, int]

    def __post_init__(self) -> None:
        if len(self.low) != 3 or len(self.high) != 3 or any(
            type(lo) is not int or type(hi) is not int or not 0 <= lo <= hi <= cap
            for lo, hi, cap in zip(self.low, self.high, (179, 255, 255))
        ):
            raise ValueError("each HSV range must fit OpenCV H 0–179 and S/V 0–255")


@dataclass(frozen=True)
class ColorClass:
    label: str
    ranges: tuple[HSVRange, ...]

    def __post_init__(self) -> None:
        if not self.label or not self.ranges:
            raise ValueError("each color requires a label and at least one HSV range")


DEFAULT_COLORS = (
    ColorClass("red", (HSVRange((0, 110, 75), (10, 255, 255)),
                       HSVRange((170, 110, 75), (179, 255, 255)))),
    ColorClass("green", (HSVRange((40, 90, 60), (85, 255, 255)),)),
)


class HSVDetector:
    def __init__(
        self, colors: tuple[ColorClass, ...] = DEFAULT_COLORS, min_area_px: int = 100,
        max_area_fraction: float = 0.55, morphology_size: int = 3,
    ) -> None:
        if min_area_px < 1 or not 0 < max_area_fraction <= 1:
            raise ValueError("invalid HSV area filters")
        if morphology_size < 0 or (morphology_size and morphology_size % 2 == 0):
            raise ValueError("morphology_size must be zero or a positive odd number")
        self._colors = [
            (color.label, [(np.array(r.low, dtype=np.uint8), np.array(r.high, dtype=np.uint8))
                           for r in color.ranges])
            for color in colors
        ]
        self._min_area = min_area_px
        self._max_fraction = max_area_fraction
        self._kernel = np.ones((morphology_size, morphology_size), dtype=np.uint8) if morphology_size else None

    def detect(self, frame: Frame) -> list[Detection]:
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        for label, ranges in self._colors:
            mask = np.zeros((height, width), dtype=np.uint8)
            for low, high in ranges:
                cv2.bitwise_or(mask, cv2.inRange(hsv, low, high), dst=mask)
            if self._kernel is not None:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) < self._min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w * h <= self._max_fraction * width * height:
                    box = BoundingBox(x, y, x + w, y + h)
                    detections.append(Detection.from_box(label, box, width, height))
        return sorted(detections, key=lambda obj: (obj.label, obj.bbox.x1, obj.bbox.y1))


class YOLODetector:
    def __init__(self, weights: str, confidence: float = 0.5) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ValueError("the YOLO backend requires the optional yolo extra") from exc

        if not 0 <= confidence <= 1:
            raise ValueError("YOLO confidence must be between 0 and 1")
        self._model = YOLO(weights)
        self._confidence = confidence

    def detect(self, frame: Frame) -> list[Detection]:
        height, width = frame.shape[:2]
        result = self._model.predict(frame, conf=self._confidence, verbose=False)[0]
        detections = []
        for prediction in result.boxes:
            x1, y1, x2, y2 = prediction.xyxy[0].tolist()
            box = BoundingBox(max(0, floor(x1)), max(0, floor(y1)),
                              min(width, ceil(x2)), min(height, ceil(y2)))
            if box.x1 < box.x2 and box.y1 < box.y2:
                detections.append(Detection.from_box(
                    result.names[int(prediction.cls.item())], box, width, height, prediction.conf.item(),
                ))
        return detections
