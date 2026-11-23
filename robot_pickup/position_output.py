"""Publish the selected target's image position for an external controller."""

import json
import logging
from contextlib import ExitStack
from pathlib import Path
from threading import Event
from time import monotonic
from typing import Callable

from robot_pickup.communication.serial_adapter import SerialArduino
from robot_pickup.config import Config
from robot_pickup.control.controller import TargetPolicy
from robot_pickup.display import DebugWindow
from robot_pickup.models import Detection, Scene, is_fresh
from robot_pickup.perception.camera import CameraSource, CameraVision

LOG = logging.getLogger(__name__)


class PositionReporter:
    interval_s = 0.1

    def __init__(self, policy: TargetPolicy, max_frame_age_s: float, emit: Callable[[str], None]) -> None:
        self._policy = policy
        self._max_frame_age_s = max_frame_age_s
        self._emit = emit
        self._found: bool | None = None
        self._last_frame_at = float("-inf")
        self._last_report_at = float("-inf")

    def update(self, scene: Scene | None, now: float) -> Detection | None:
        if scene is None or not is_fresh(scene.observed_at, now, self._max_frame_age_s):
            self.clear()
            return None

        target = self._policy.select(scene, aim_x=0.0)
        if target is None:
            self.clear()
            return None

        if scene.observed_at > self._last_frame_at and now - self._last_report_at >= self.interval_s:
            box = target.bbox
            message = {
                "type": "target",
                "found": True,
                "label": target.label,
                "object_id": target.object_id,
                "confidence": round(target.confidence, 4) if target.confidence is not None else None,
                "cx": round(target.cx, 4),
                "cy": round(target.cy, 4),
                "x_px": (box.x1 + box.x2) / 2,
                "y_px": (box.y1 + box.y2) / 2,
                "frame_width": scene.width,
                "frame_height": scene.height,
            }
            self._emit(json.dumps(message, separators=(",", ":"), allow_nan=False))
            self._found = True
            self._last_frame_at = scene.observed_at
            self._last_report_at = now
        return target

    def clear(self) -> None:
        if self._found is not False:
            self._emit('{"type":"target","found":false}')
            self._found = False


def run_position_output(config: Config, source: CameraSource, display: bool, serial_port: str | None) -> int:
    backend = config.perception.backend.upper()
    if config.perception.backend == "yolo":
        backend += f" ({Path(config.perception.yolo_weights).name})"
    LOG.info("Target position output using %s", backend)
    vision = CameraVision(source, config.camera, config.perception.pipeline())
    pace = Event()

    with ExitStack() as stack:
        serial = None
        if serial_port is not None:
            settings = config.serial
            LOG.info("Sending target positions on %s at %d baud", serial_port, settings.baudrate)
            serial = SerialArduino.open(serial_port, settings.baudrate, settings.write_timeout_s, False)
            stack.callback(serial.close)
            pace.wait(settings.startup_s)

        def emit(line: str) -> None:
            if serial is not None:
                serial.write_line(line)
            print(line, flush=True)

        reporter = PositionReporter(config.target, config.control.frame_timeout_s, emit)
        vision.start()
        stack.callback(vision.close)
        window = DebugWindow(f"Target position - {backend}") if display else None
        if window is not None:
            stack.callback(window.close)
        deadline = monotonic() + config.camera.startup_timeout_s

        try:
            while True:
                finished = vision.finished.is_set()
                snapshot = vision.latest
                now = monotonic()
                if serial is not None:
                    serial.poll(now)
                target = reporter.update(snapshot.scene if snapshot is not None else None, now)
                if window is not None and snapshot is not None and not window.show_position(snapshot, target):
                    return 0
                if finished:
                    if vision.error:
                        LOG.error("%s", vision.error)
                        return 2
                    return 0
                if snapshot is None and now >= deadline:
                    LOG.error("No processed camera frame before the startup timeout")
                    return 2
                pace.wait(max(0, now + 1 / config.runtime.tick_hz - monotonic()))
        finally:
            try:
                reporter.clear()
            except OSError as exc:
                LOG.warning("Could not send the final target status (%s)", exc)
