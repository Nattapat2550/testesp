import logging
from dataclasses import dataclass
from enum import Enum
from itertools import count
from typing import Callable, Protocol

LOG = logging.getLogger(__name__)


class CommandKind(str, Enum):
    MOVE = "MOVE"
    TURN = "TURN"
    VELOCITY = "VELOCITY"
    STOP = "STOP"
    LOWER = "LOWER"
    GRAB = "GRAB"
    LIFT = "LIFT"
    RELEASE = "RELEASE"
    HEARTBEAT = "HEARTBEAT"


BASE_COMMANDS = {CommandKind.MOVE, CommandKind.TURN, CommandKind.VELOCITY, CommandKind.STOP}
ARM_COMMANDS = {CommandKind.LOWER, CommandKind.GRAB, CommandKind.LIFT, CommandKind.RELEASE}


@dataclass(frozen=True)
class Command:
    kind: CommandKind
    linear: float = 0.0
    angular: float = 0.0

    def __post_init__(self) -> None:
        if not (-1 <= self.linear <= 1 and -1 <= self.angular <= 1):
            raise ValueError("command speeds must be between -1 and 1")
        if self.linear and self.kind not in (CommandKind.MOVE, CommandKind.VELOCITY):
            raise ValueError("only MOVE and VELOCITY accept linear speed")
        if self.angular and self.kind not in (CommandKind.TURN, CommandKind.VELOCITY):
            raise ValueError("only TURN and VELOCITY accept angular speed")


STOP = Command(CommandKind.STOP)
HEARTBEAT = Command(CommandKind.HEARTBEAT)


@dataclass(frozen=True)
class Transmission:
    sequence: int
    command: Command
    sent_at: float


@dataclass(frozen=True)
class Ack:
    sequence: int
    accepted: bool


@dataclass(frozen=True)
class ActionDone:
    sequence: int
    success: bool


@dataclass(frozen=True)
class GripFeedback:
    sequence: int
    holding: bool
    measured_at: float


Event = Ack | ActionDone | GripFeedback


class CommandTransport(Protocol):
    supports_combined_velocity: bool

    def send(self, transmission: Transmission) -> None: ...

    def poll(self, now: float) -> list[Event]:
        """Return available events without waiting for an acknowledgment."""

    def close(self) -> None: ...


@dataclass(frozen=True)
class LinkSettings:
    ack_timeout_s: float = 0.35
    heartbeat_s: float = 0.12

    def __post_init__(self) -> None:
        if not 0 < self.heartbeat_s < self.ack_timeout_s:
            raise ValueError("heartbeat interval must be positive and shorter than the ACK timeout")


class CommandLink:
    """Track acknowledgments and arm completions and deduplicate base commands."""

    def __init__(self, transport: CommandTransport, settings: LinkSettings) -> None:
        self.transport = transport
        self.settings = settings
        self.fault: str | None = None
        self.last_motion: Command | None = None
        self.grip: GripFeedback | None = None
        self._sequences = count(1)
        self._pending: dict[int, Transmission] = {}
        self._last_send = float("-inf")
        self._base_sequence: int | None = None
        self._action_sequence: int | None = None
        self._done: int | None = None

    @property
    def stopped(self) -> bool:
        return self.fault is None and self.last_motion == STOP and self._base_sequence not in self._pending

    def poll(self, now: float) -> None:
        if self.fault is not None:
            return
        try:
            events = self.transport.poll(now)
        except OSError:
            self.fault = "Arduino read failed"
            return
        for pending in self._pending.values():
            if now - pending.sent_at >= self.settings.ack_timeout_s:
                self.fault = f"Arduino ACK timeout for {pending.command.kind.value}"
                return
        for event in events:
            if isinstance(event, Ack) and event.sequence in self._pending:
                pending = self._pending.pop(event.sequence)
                if not event.accepted:
                    self.fault = f"Arduino rejected {pending.command.kind.value}"
                    return
            elif isinstance(event, ActionDone) and event.sequence == self._action_sequence:
                if not event.success:
                    self.fault = "Arduino reported an arm action failure"
                    return
                self._done = event.sequence
            elif isinstance(event, GripFeedback) and event.sequence == self._action_sequence:
                if self.grip is None or event.measured_at >= self.grip.measured_at:
                    self.grip = event

    def _send(self, command: Command, now: float) -> int | None:
        if self.fault is not None:
            return None
        transmission = Transmission(next(self._sequences), command, now)
        try:
            self.transport.send(transmission)
        except OSError:
            self.fault = f"Arduino write failed for {command.kind.value}"
            return None
        self._pending[transmission.sequence] = transmission
        self._last_send = now
        LOG.debug("Sent %s", transmission)
        return transmission.sequence

    def set_motion(self, command: Command, now: float) -> None:
        if command.kind not in BASE_COMMANDS:
            raise ValueError("expected a base command")
        if command == self.last_motion:
            return
        if command != STOP and self._pending:
            return
        sequence = self._send(command, now)
        if sequence is not None:
            self.last_motion = command
            self._base_sequence = sequence

    def start_action(self, command: CommandKind, now: float) -> int | None:
        if command not in ARM_COMMANDS:
            raise ValueError("expected an arm command")
        if not self.stopped or self._pending:
            return None
        if self._action_sequence is not None and not self.action_done(self._action_sequence):
            return None
        sequence = self._send(Command(command), now)
        if sequence is not None:
            self._action_sequence = sequence
            self._done = None
            self.grip = None
        return sequence

    def action_done(self, sequence: int) -> bool:
        return self._done == sequence and sequence not in self._pending

    def keepalive(self, now: float) -> None:
        # A heartbeat must not renew old motion while a STOP or another command is unacknowledged.
        if not self._pending and now - self._last_send >= self.settings.heartbeat_s:
            self._send(HEARTBEAT, now)

    def emergency_stop(self, now: float) -> None:
        """Make one immediate STOP attempt, even if the link already faulted."""
        try:
            self.transport.send(Transmission(next(self._sequences), STOP, now))
        except OSError:
            LOG.error("Emergency STOP write failed")
        else:
            self.last_motion = STOP


class MockArduino:
    def __init__(
        self, action_time_s: float = 0.12, on_action: Callable[[CommandKind], bool | None] | None = None,
        supports_combined_velocity: bool = True,
    ) -> None:
        self.supports_combined_velocity = supports_combined_velocity
        self.history: list[Transmission] = []
        self.motion = STOP
        self._events: list[Event] = []
        self._action: Transmission | None = None
        self._action_time = action_time_s
        self._on_action = on_action

    def send(self, transmission: Transmission) -> None:
        self.history.append(transmission)
        command = transmission.command
        self._events.append(Ack(transmission.sequence, True))
        if command.kind in BASE_COMMANDS:
            self.motion = command
        if command == STOP:
            self._action = None
        elif command.kind in ARM_COMMANDS:
            self._action = transmission

    def poll(self, now: float) -> list[Event]:
        if self._action is not None and now - self._action.sent_at >= self._action_time:
            action, self._action = self._action, None
            self._events.append(ActionDone(action.sequence, True))
            holding = self._on_action(action.command.kind) if self._on_action is not None else None
            if holding is not None:
                self._events.append(GripFeedback(action.sequence, holding, now))
        events, self._events = self._events, []
        return events

    def close(self) -> None:
        pass
