"""Adapter for the JSON-lines protocol proposed in docs/arduino-protocol.md."""

import json
from typing import Protocol

from robot_pickup.communication.commands import Ack, ActionDone, Event, GripFeedback, Transmission
from robot_pickup.models import DistanceSample, finite_number
from robot_pickup.sensors.distance import parse_distance


class SerialPort(Protocol):
    @property
    def in_waiting(self) -> int: ...
    def write(self, data: bytes) -> int: ...
    def read(self, size: int) -> bytes: ...
    def close(self) -> None: ...


class SerialArduino:
    MAX_LINE_BYTES = 1024
    MAX_TICK_BYTES = 4096

    def __init__(self, port: SerialPort, supports_combined_velocity: bool = False) -> None:
        self.supports_combined_velocity = supports_combined_velocity
        self._port = port
        self._partial = b""
        self._discarding = False
        self._distance: DistanceSample | None = None
        self._last_sample_id = -1

    @classmethod
    def open(
        cls, port: str, baudrate: int, write_timeout_s: float, supports_combined_velocity: bool,
    ) -> "SerialArduino":
        try:
            import serial
        except ImportError as exc:
            raise ValueError("serial access requires the optional serial extra") from exc

        connection = serial.Serial(port, baudrate, timeout=0, write_timeout=write_timeout_s)
        try:
            connection.reset_input_buffer()
        except OSError:
            connection.close()
            raise
        return cls(connection, supports_combined_velocity)

    def send(self, transmission: Transmission) -> None:
        command = transmission.command
        message = {"type": "cmd", "seq": transmission.sequence, "command": command.kind.value,
                   "linear": command.linear, "angular": command.angular}
        self.write_line(json.dumps(message, separators=(",", ":"), allow_nan=False))

    def write_line(self, line: str) -> None:
        wire = (line + "\n").encode("ascii")
        if self._port.write(wire) != len(wire):
            raise OSError("incomplete Arduino serial write")

    def poll(self, now: float) -> list[Event]:
        waiting = self._port.in_waiting
        if waiting > self.MAX_TICK_BYTES:
            raise OSError("Arduino receive backlog exceeded")
        if not waiting:
            return []
        lines = (self._partial + self._port.read(waiting)).split(b"\n")
        self._partial = lines.pop()
        if self._discarding and lines:
            lines.pop(0)
            self._discarding = False
        if len(self._partial) > self.MAX_LINE_BYTES:
            self._partial = b""
            self._discarding = True

        events = []
        for line in lines:
            if len(line) > self.MAX_LINE_BYTES:
                continue
            try:
                message = json.loads(line)
            except (UnicodeError, ValueError):
                continue
            if isinstance(message, dict):
                event = self._message(message, now)
                if event is not None:
                    events.append(event)
        return events

    def _message(self, message: dict[str, object], now: float) -> Event | None:
        kind = message.get("type")
        sequence = message.get("seq")
        if kind == "ack" and type(sequence) is int and type(message.get("accepted")) is bool:
            return Ack(sequence, message["accepted"])
        if kind == "done" and type(sequence) is int and type(message.get("success")) is bool:
            return ActionDone(sequence, message["success"])

        age_ms = message.get("age_ms")
        if kind == "distance":
            sample_id = message.get("sample_id")
            if type(sample_id) is not int or sample_id < 0 or not finite_number(age_ms) or age_ms < 0:
                self._distance = DistanceSample(None, now, False, "invalid serial distance timestamp")
            elif sample_id == self._last_sample_id:
                if message.get("valid") is not True or not finite_number(message.get("distance_cm")):
                    self._distance = DistanceSample(None, now, False, "invalid repeated distance telemetry")
            elif sample_id < self._last_sample_id:
                self._distance = DistanceSample(None, now, False, "out-of-order serial distance timestamp")
            else:
                self._last_sample_id = sample_id
                self._distance = parse_distance(message, now - age_ms / 1000)
        elif kind == "grip" and type(sequence) is int and type(message.get("holding")) is bool:
            if finite_number(age_ms) and age_ms >= 0:
                return GripFeedback(sequence, message["holding"], now - age_ms / 1000)
        return None

    def latest(self, now: float) -> DistanceSample | None:
        return self._distance

    def close(self) -> None:
        self._port.close()
