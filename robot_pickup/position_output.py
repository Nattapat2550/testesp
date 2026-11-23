"""Publish the selected target's image position for an external controller."""

import json
import logging
from contextlib import ExitStack
from dataclasses import asdict, dataclass
from functools import partial
from hashlib import sha256
from pathlib import Path
from tempfile import NamedTemporaryFile
from threading import Event
from time import monotonic
from typing import Callable

from robot_pickup.communication.serial_adapter import SerialArduino
from robot_pickup.config import Config
from robot_pickup.control.controller import TargetPolicy
from robot_pickup.display import DebugWindow
from robot_pickup.models import Detection, Scene, finite_number, is_fresh
from robot_pickup.perception.camera import CameraSource, CameraVision

LOG = logging.getLogger(__name__)
PICKUP_POINT_PATH = Path("pickup-point.json")


@dataclass(frozen=True)
class PickupPoint:
    label: str
    pickup_cx: float
    pickup_cy: float
    frame_width: int
    frame_height: int
    camera_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.label, str) or not self.label:
            raise ValueError("invalid pickup point label")
        if any(not finite_number(value) or not -1 <= value <= 1 for value in (self.pickup_cx, self.pickup_cy)):
            raise ValueError("invalid pickup point coordinates")
        if any(type(value) is not int or value <= 0 for value in (self.frame_width, self.frame_height)):
            raise ValueError("invalid pickup point frame dimensions")

    def matches(self, scene: Scene, target: Detection) -> bool:
        return (self.label, self.frame_width, self.frame_height) == (target.label, scene.width, scene.height)


def load_pickup_point(camera_id: str) -> PickupPoint | None:
    try:
        if PICKUP_POINT_PATH.is_symlink() or not PICKUP_POINT_PATH.is_file():
            return None
        with PICKUP_POINT_PATH.open(encoding="utf-8") as file:
            point = PickupPoint(**json.load(file))
    except (OSError, TypeError, ValueError):
        LOG.warning("Saved pickup point is unavailable. Press P to teach this camera.")
        return None
    if point.camera_id != camera_id:
        LOG.warning("Saved pickup point belongs to another camera source. Press P to teach this camera.")
        return None
    return point


def save_pickup_point(
    scene: Scene, target: Detection | None, max_frame_age_s: float, camera_id: str,
) -> PickupPoint | None:
    if target is None or not is_fresh(scene.observed_at, monotonic(), max_frame_age_s):
        LOG.warning("Show a visible target at the pickup spot before pressing P")
        return None

    point = PickupPoint(target.label, round(target.cx, 4), round(target.cy, 4), scene.width, scene.height, camera_id)
    temporary_path = None
    try:
        with NamedTemporaryFile("w", encoding="utf-8", dir=PICKUP_POINT_PATH.parent, delete=False) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(asdict(point), temporary, indent=2, allow_nan=False)
            temporary.write("\n")
        temporary_path.replace(PICKUP_POINT_PATH)
        temporary_path = None
    except (OSError, ValueError) as exc:
        LOG.error("Could not save the pickup point (%s)", exc)
        return None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError as exc:
                LOG.warning("Could not clean up the pickup point temporary file (%s)", exc)
    LOG.info("Saved pickup point to %s (cx=%+.4f, cy=%+.4f)",
             PICKUP_POINT_PATH, point.pickup_cx, point.pickup_cy)
    return point


class PositionReporter:
    interval_s = 0.1

    def __init__(
        self, policy: TargetPolicy, max_frame_age_s: float, emit: Callable[[str], None],
        pickup_point: PickupPoint | None = None,
    ) -> None:
        self._policy = policy
        self._max_frame_age_s = max_frame_age_s
        self._emit = emit
        self._pickup_point = pickup_point
        self._found: bool | None = None
        self._last_frame_at = float("-inf")
        self._last_report_at = float("-inf")

    def teach_pickup(self, scene: Scene, target: Detection | None, camera_id: str) -> None:
        point = save_pickup_point(scene, target, self._max_frame_age_s, camera_id)
        if point is not None:
            self._pickup_point = point

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
            point = self._pickup_point
            if point is not None and point.matches(scene, target):
                message.update(pickup_cx=point.pickup_cx, pickup_cy=point.pickup_cy)
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
    camera_id = sha256(json.dumps([source.address, source.is_stream]).encode("ascii")).hexdigest()
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

        reporter = PositionReporter(config.target, config.control.frame_timeout_s, emit, load_pickup_point(camera_id))
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
                if window is not None and snapshot is not None:
                    on_save = partial(reporter.teach_pickup, snapshot.scene, target, camera_id)
                    if not window.show_position(snapshot, target, on_save):
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
