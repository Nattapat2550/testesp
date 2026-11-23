from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from threading import Event, Thread
from time import monotonic
from urllib.parse import urlsplit

import cv2

from robot_pickup.models import Scene
from robot_pickup.perception.detector import Frame
from robot_pickup.perception.tracking import VisionPipeline


@dataclass(frozen=True)
class CameraSettings:
    open_timeout_ms: int = 3000
    read_timeout_ms: int = 1000
    startup_timeout_s: float = 8.0

    def __post_init__(self) -> None:
        if any(type(value) is not int or value <= 0 for value in (self.open_timeout_ms, self.read_timeout_ms)):
            raise ValueError("camera I/O timeouts must be positive integers in milliseconds")
        if not 0 < self.startup_timeout_s:
            raise ValueError("camera startup timeout must be positive")


@dataclass(frozen=True)
class CameraSource:
    address: int | str
    is_stream: bool = False

    @property
    def is_live(self) -> bool:
        return self.is_stream or isinstance(self.address, int)

    @classmethod
    def parse(cls, source: str) -> "CameraSource":
        if source.isdecimal():
            return cls(int(source))
        if source.lower().startswith(("http://", "https://")):
            if not urlsplit(source).hostname:
                raise ValueError("invalid camera stream URL")
            return cls(source, is_stream=True)
        if not Path(source).is_file():
            raise ValueError("camera source must be a webcam index, an HTTP stream URL, or an existing video file")
        return cls(source)


@dataclass(frozen=True)
class VisionSnapshot:
    frame: Frame
    scene: Scene


class CameraVision:
    """Read and process frames in a worker so camera I/O cannot stall control ticks."""

    def __init__(self, source: CameraSource, settings: CameraSettings, pipeline: VisionPipeline) -> None:
        self._source = source
        self._settings = settings
        self._pipeline = pipeline
        self.latest: VisionSnapshot | None = None
        self.error: str | None = None
        self.finished = Event()
        self._stop = Event()
        self._worker = Thread(target=self._run, name="camera-vision", daemon=True)

    def start(self) -> None:
        self._worker.start()

    def close(self) -> None:
        self._stop.set()
        io_timeout = max(self._settings.open_timeout_ms, self._settings.read_timeout_ms) / 1000
        self._worker.join(timeout=io_timeout + 0.25 if self._source.is_stream else 0.75)

    def _open(self) -> cv2.VideoCapture:
        if self._source.is_stream:
            return cv2.VideoCapture(self._source.address, cv2.CAP_FFMPEG, [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self._settings.open_timeout_ms,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC, self._settings.read_timeout_ms,
            ])
        return cv2.VideoCapture(self._source.address)

    def _run(self) -> None:
        capture = None
        try:
            capture = self._open()
            if not capture.isOpened():
                self.error = "could not open the camera or video source"
                return
            fps = capture.get(cv2.CAP_PROP_FPS)
            playback_interval = 0.0 if self._source.is_live else 1 / (fps if isfinite(fps) and fps > 0 else 30)
            while not self._stop.is_set():
                started_at = monotonic()
                ok, frame = capture.read()
                if not ok or frame is None:
                    if self._source.is_live or self.latest is None:
                        self.error = "camera stopped delivering frames"
                    return
                scene = self._pipeline.process(frame, monotonic())
                self.latest = VisionSnapshot(frame, scene)
                if playback_interval:
                    self._stop.wait(max(0, started_at + playback_interval - monotonic()))
        except Exception as exc:
            self.error = f"camera or detector failed ({type(exc).__name__})"
        finally:
            if capture is not None:
                capture.release()
            self.finished.set()
