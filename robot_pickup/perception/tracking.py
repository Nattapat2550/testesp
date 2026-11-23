from dataclasses import replace
from itertools import count
from math import hypot

from robot_pickup.models import Detection, Scene, is_fresh
from robot_pickup.perception.detector import Detector, Frame


class CentroidTracker:
    """Match same-label objects one-to-one within a normalized distance gate."""

    def __init__(self, max_displacement: float = 0.22, max_gap_s: float = 0.6) -> None:
        if not 0 < max_displacement <= 2 or not 0 < max_gap_s:
            raise ValueError("invalid tracker limits")
        self._max_displacement = max_displacement
        self._max_gap = max_gap_s
        self._ids = count(1)
        self._tracks: dict[int, tuple[Detection, float]] = {}

    def update(self, detections: list[Detection], now: float) -> tuple[Detection, ...]:
        self._tracks = {
            key: (obj, seen) for key, (obj, seen) in self._tracks.items()
            if is_fresh(seen, now, self._max_gap)
        }
        pairs = []
        for index, obj in enumerate(detections):
            for key, (previous, _) in self._tracks.items():
                if obj.label != previous.label:
                    continue
                distance = hypot(obj.cx - previous.cx, obj.cy - previous.cy)
                if distance <= self._max_displacement:
                    pairs.append((distance, key, index))

        matches: dict[int, int] = {}
        assigned = set()
        for _, key, index in sorted(pairs):
            if key not in assigned and index not in matches:
                matches[index] = key
                assigned.add(key)

        visible = []
        for index, obj in enumerate(detections):
            key = matches[index] if index in matches else next(self._ids)
            tracked = replace(obj, object_id=key)
            self._tracks[key] = tracked, now
            visible.append(tracked)
        return tuple(visible)


class VisionPipeline:
    def __init__(self, detector: Detector, tracker: CentroidTracker) -> None:
        self._detector = detector
        self._tracker = tracker

    def process(self, frame: Frame, observed_at: float) -> Scene:
        height, width = frame.shape[:2]
        detections = self._tracker.update(self._detector.detect(frame), observed_at)
        return Scene(observed_at, width, height, detections)
