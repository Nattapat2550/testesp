import json
import logging
from collections import deque
from dataclasses import dataclass
from http.client import HTTPException
from statistics import median
from threading import Event, Thread
from time import monotonic, time
from typing import Callable, Protocol
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from robot_pickup.models import DistanceReading, DistanceSample, finite_number, is_fresh

LOG = logging.getLogger(__name__)


class DistanceProvider(Protocol):
    def latest(self, now: float) -> DistanceSample | None:
        """Return the latest measurement without waiting for I/O."""


class NoDistance:
    def latest(self, now: float) -> None:
        return None


class SimulatedDistance:
    def __init__(self, measure: Callable[[], float | None], interval_s: float = 0.1) -> None:
        if not 0 < interval_s:
            raise ValueError("simulation sample interval must be positive")
        self._measure = measure
        self._interval = interval_s
        self._sample: DistanceSample | None = None

    def latest(self, now: float) -> DistanceSample:
        if self._sample is None or now - self._sample.measured_at >= self._interval:
            cm = self._measure()
            self._sample = DistanceSample(cm, now, cm is not None, "no echo" if cm is None else "")
        return self._sample


@dataclass(frozen=True)
class DistanceSettings:
    min_cm: float = 2.0
    max_cm: float = 400.0
    max_age_s: float = 0.35
    median_window: int = 3

    def __post_init__(self) -> None:
        if not 0 < self.min_cm < self.max_cm or not 0 < self.max_age_s:
            raise ValueError("invalid distance or freshness limits")
        if type(self.median_window) is not int or self.median_window < 1:
            raise ValueError("median_window must be a positive integer")


class FilteredDistance:
    def __init__(self, provider: DistanceProvider, settings: DistanceSettings) -> None:
        self._provider = provider
        self._settings = settings
        self._window: deque[DistanceSample] = deque(maxlen=settings.median_window)
        self._last_sample: DistanceSample | None = None

    def latest(self, now: float) -> DistanceReading:
        sample = self._provider.latest(now)
        limits = self._settings
        if sample is None:
            reason = "no distance data"
        elif not is_fresh(sample.measured_at, now, limits.max_age_s):
            reason = "distance timestamp is stale or invalid"
        elif self._last_sample is not None and sample != self._last_sample and sample.measured_at <= self._last_sample.measured_at:
            reason = "distance samples are out of order"
        else:
            self._last_sample = sample
            if not sample.valid:
                reason = sample.reason or "invalid distance sample"
            elif not finite_number(sample.cm) or not limits.min_cm <= sample.cm <= limits.max_cm:
                reason = "distance is outside configured limits"
            else:
                while self._window and not is_fresh(self._window[0].measured_at, now, limits.max_age_s):
                    self._window.popleft()
                if not self._window or sample.measured_at > self._window[-1].measured_at:
                    self._window.append(sample)
                return DistanceReading(sample, median(item.cm for item in self._window), "")
        self._window.clear()
        return DistanceReading(sample, reason=reason)


def parse_distance(payload: dict[str, object], measured_at: float) -> DistanceSample:
    cm = payload.get("distance_cm")
    valid = payload.get("valid") is True and finite_number(cm)
    return DistanceSample(float(cm) if valid else None, measured_at, valid,
                          "" if valid else "invalid distance telemetry")


class HTTPDistance:
    """Poll a timestamped JSON endpoint without blocking the controller."""

    def __init__(self, url: str, timeout_s: float = 0.25, interval_s: float = 0.1) -> None:
        parsed = urlsplit(url)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise ValueError("distance URL must be an HTTP or HTTPS endpoint")
        if not 0 < timeout_s or not 0 < interval_s:
            raise ValueError("HTTP timeout and poll interval must be positive")
        self._request = Request(url, headers={"Accept": "application/json", "Cache-Control": "no-cache"})
        self._timeout = timeout_s
        self._interval = interval_s
        self._samples: tuple[DistanceSample | None, DistanceSample | None] = None, None
        self._last_source_timestamp: float | None = None
        self._stop = Event()
        self._worker = Thread(target=self._run, name="distance-http", daemon=True)

    def start(self) -> None:
        self._worker.start()

    def latest(self, now: float) -> DistanceSample | None:
        current, previous = self._samples
        return current if current is not None and current.measured_at <= now else previous

    def close(self) -> None:
        self._stop.set()
        self._worker.join(self._timeout + 0.1)

    def _publish(self, sample: DistanceSample) -> None:
        self._samples = sample, self._samples[0]

    def _receive(self, payload: object, now: float, wall_time: float) -> None:
        timestamp = payload.get("measured_at") if isinstance(payload, dict) else None
        if not finite_number(timestamp) or not 0 <= timestamp <= wall_time:
            raise ValueError("distance endpoint requires a valid Unix measurement timestamp")
        if self._last_source_timestamp is not None and timestamp <= self._last_source_timestamp:
            if (timestamp < self._last_source_timestamp or payload.get("valid") is not True
                    or not finite_number(payload.get("distance_cm"))):
                self._publish(DistanceSample(None, now, False, "invalid or out-of-order HTTP distance"))
            return
        self._last_source_timestamp = timestamp
        self._publish(parse_distance(payload, now - (wall_time - timestamp)))

    def _run(self) -> None:
        while not self._stop.is_set():
            previous = self._samples[0]
            try:
                with urlopen(self._request, timeout=self._timeout) as response:
                    body = response.read(4097)
                if len(body) > 4096:
                    raise ValueError("oversized distance response")
                self._receive(json.loads(body), monotonic(), time())
            except (OSError, ValueError, HTTPException) as exc:
                self._publish(DistanceSample(None, monotonic(), False, "HTTP distance unavailable"))
                if previous is None or previous.valid:
                    LOG.warning("Distance endpoint unavailable (%s)", type(exc).__name__)
            self._stop.wait(self._interval)
