import logging
from dataclasses import dataclass
from enum import Enum
from math import copysign, isfinite

from robot_pickup.communication.commands import ARM_COMMANDS, STOP, Command, CommandKind, CommandLink
from robot_pickup.models import Detection, DistanceReading, Scene, is_fresh
from robot_pickup.sensors.association import TargetRanger
from robot_pickup.sensors.distance import FilteredDistance

LOG = logging.getLogger(__name__)


class State(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    SELECT = "SELECT"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    PICKUP = "PICKUP"
    VERIFY = "VERIFY"
    COMPLETE = "COMPLETE"
    RECOVER = "RECOVER"


class PickupResult(str, Enum):
    VERIFIED = "verified"
    UNVERIFIED = "unverified"
    FAILED = "failed"


@dataclass(frozen=True)
class TargetPolicy:
    labels: tuple[str, ...] = ()
    strategy: str = "centered"
    min_confidence: float = 0.5

    def __post_init__(self) -> None:
        if self.strategy not in ("centered", "largest") or not 0 <= self.min_confidence <= 1:
            raise ValueError("invalid target strategy or confidence threshold")

    def eligible(self, obj: Detection) -> bool:
        return (
            obj.object_id is not None and (not self.labels or obj.label in self.labels)
            and (obj.confidence is None or obj.confidence >= self.min_confidence)
        )

    def select(self, scene: Scene, aim_x: float) -> Detection | None:
        def rank(obj: Detection) -> tuple[float, float]:
            center = abs(obj.cx - aim_x)
            area = -obj.area_fraction
            return (center, area) if self.strategy == "centered" else (area, center)

        return min((obj for obj in scene.objects if self.eligible(obj)), key=rank, default=None)


@dataclass(frozen=True)
class ControllerSettings:
    center_tolerance: float = 0.06
    realign_tolerance: float = 0.12
    max_linear: float = 0.30
    max_angular: float = 0.42
    search_angular: float = 0.0
    turn_gain: float = 1.8
    approach_gain_per_cm: float = 0.05
    pickup_cm: float = 12.0
    pickup_window_cm: float = 3.0
    resume_margin_cm: float = 1.0
    settle_s: float = 0.10
    pickup_samples: int = 2
    frame_timeout_s: float = 0.40
    target_timeout_s: float = 0.65
    search_timeout_s: float = 8.0
    motion_timeout_s: float = 12.0
    pickup_timeout_s: float = 8.0
    verify_timeout_s: float = 1.0
    require_range_for_turn: bool = True
    pickup_sequence: tuple[CommandKind, ...] = (CommandKind.LOWER, CommandKind.GRAB, CommandKind.LIFT)

    def __post_init__(self) -> None:
        if not 0 < self.center_tolerance < self.realign_tolerance <= 1:
            raise ValueError("center tolerance must be smaller than realign tolerance")
        if not (0 < self.max_linear <= 1 and 0 < self.max_angular <= 1
                and abs(self.search_angular) <= self.max_angular):
            raise ValueError("invalid normalized speed limits")
        if not 0 < self.pickup_window_cm < self.pickup_cm or not isfinite(self.pickup_cm):
            raise ValueError("invalid pickup distance window")
        positive = (
            self.turn_gain, self.approach_gain_per_cm, self.resume_margin_cm, self.settle_s,
            self.frame_timeout_s, self.target_timeout_s, self.search_timeout_s, self.motion_timeout_s,
            self.pickup_timeout_s, self.verify_timeout_s,
        )
        if any(not isfinite(value) or value <= 0 for value in positive):
            raise ValueError("controller gains, timeouts, and hysteresis must be positive and finite")
        if type(self.pickup_samples) is not int or self.pickup_samples < 1:
            raise ValueError("pickup_samples must be a positive integer")
        if type(self.require_range_for_turn) is not bool:
            raise ValueError("require_range_for_turn must be a boolean")
        if not self.pickup_sequence or any(command not in ARM_COMMANDS for command in self.pickup_sequence):
            raise ValueError("pickup_sequence must contain arm commands")

    @property
    def pickup_floor_cm(self) -> float:
        return self.pickup_cm - self.pickup_window_cm


@dataclass(frozen=True)
class ControlStatus:
    state: State
    target_id: int | None = None
    target_visible: bool = False
    target_distance_cm: float | None = None
    motion: Command | None = None
    action: CommandKind | None = None
    result: PickupResult | None = None
    reason: str = ""
    beam_cm: float | None = None


class PickupController:
    """Advance at most one state at a time using current, timestamped observations."""

    def __init__(
        self, link: CommandLink, distance: FilteredDistance, ranger: TargetRanger,
        settings: ControllerSettings, policy: TargetPolicy,
    ) -> None:
        self.settings = settings
        self._link = link
        self._distance = distance
        self._ranger = ranger
        self._policy = policy
        self.state = State.IDLE
        self.result: PickupResult | None = None
        self.reason = ""
        self._state_since = 0.0
        self._search_since = 0.0
        self._motion_since = 0.0
        self._target_id: int | None = None
        self._target_seen = 0.0
        self._aligned_at: float | None = None
        self._braking = False
        self._stopped_at: float | None = None
        self._confirmations = 0
        self._confirmed_at = float("-inf")
        self._action_index = 0
        self._action_sequence: int | None = None
        self._action_sent_at = 0.0

    @property
    def terminal(self) -> bool:
        return self.state in (State.COMPLETE, State.RECOVER)

    def activate(self, now: float) -> None:
        if self.state is State.IDLE:
            self._search_since = now
            self._transition(State.SEARCH, now)
            self._link.set_motion(STOP, now)

    def abort(self, now: float, reason: str) -> None:
        if self.state is not State.IDLE and not self.terminal:
            self._link.emergency_stop(now)
            self.result, self.reason = PickupResult.FAILED, reason
            self._transition(State.RECOVER, now)
            LOG.warning("Pickup aborted: %s", reason)

    def tick(self, scene: Scene | None, now: float) -> ControlStatus:
        self._link.poll(now)
        if self.state is State.IDLE:
            return self._status(beam_cm=self._distance.latest(now).raw_cm)
        if self.terminal:
            return self._status()
        if self._link.fault is not None:
            self.abort(now, self._link.fault)
            return self._status()
        if scene is None or not is_fresh(scene.observed_at, now, self.settings.frame_timeout_s):
            self.abort(now, "camera frame is missing or stale")
            return self._status()

        reading = self._distance.latest(now)
        target = next((obj for obj in scene.objects
                       if obj.object_id == self._target_id and self._policy.eligible(obj)), None)
        target_cm = self._ranger.estimate(target, scene, reading) if target is not None else None
        clearance = self._clearance(reading)

        if self.state in (State.SEARCH, State.SELECT):
            target = self._search(scene, clearance, now)
            if target is not None:
                target_cm = self._ranger.estimate(target, scene, reading)
        elif self.state in (State.ALIGN, State.APPROACH):
            self._navigate(target, scene, reading, target_cm, clearance, now)
        elif self.state is State.PICKUP:
            self._pickup(target, reading, target_cm, now)
        elif self.state is State.VERIFY:
            self._verify(now)

        if not self.terminal:
            self._link.keepalive(now)
            if self._link.fault is not None:
                self.abort(now, self._link.fault)
        return self._status(target, target_cm, reading.raw_cm)

    def _transition(self, state: State, now: float) -> None:
        LOG.info("%s -> %s", self.state.value, state.value)
        self.state, self._state_since = state, now

    def _status(self, target: Detection | None = None, target_cm: float | None = None,
                beam_cm: float | None = None) -> ControlStatus:
        action = self.settings.pickup_sequence[self._action_index] if self.state is State.PICKUP else None
        return ControlStatus(self.state, self._target_id, target is not None, target_cm,
                             self._link.last_motion, action, self.result, self.reason or self._link.fault or "", beam_cm)

    def _search(self, scene: Scene, clearance: float | None, now: float) -> Detection | None:
        if now - self._search_since >= self.settings.search_timeout_s:
            self.abort(now, "search timed out")
            return None
        candidate = self._policy.select(scene, self._ranger.aim_x)
        if self.state is State.SEARCH:
            if candidate is None:
                self._turn_or_hold(self.settings.search_angular, clearance, now)
            else:
                self._link.set_motion(STOP, now)
                self._transition(State.SELECT, now)
        elif candidate is None:
            self._transition(State.SEARCH, now)
        elif self._link.stopped:
            self._target_id = candidate.object_id
            self._target_seen = scene.observed_at
            self._motion_since = now
            self._realign(now)
            return candidate
        return None

    def _realign(self, now: float) -> None:
        self._link.set_motion(STOP, now)
        self._aligned_at = None
        self._reset_braking()
        self._transition(State.ALIGN, now)

    def _navigate(
        self, target: Detection | None, scene: Scene, reading: DistanceReading,
        target_cm: float | None, clearance: float | None, now: float,
    ) -> None:
        if now - self._motion_since >= self.settings.motion_timeout_s:
            self.abort(now, "alignment or approach timed out")
            return
        if target is None:
            if now - self._target_seen >= self.settings.target_timeout_s:
                self.abort(now, "selected target was lost")
            elif self.state is State.APPROACH:
                self._realign(now)
            else:
                self._aligned_at = None
                self._link.set_motion(STOP, now)
            return

        self._target_seen = scene.observed_at
        error = target.cx - self._ranger.aim_x
        if self.state is State.ALIGN:
            self._align(error, reading, target_cm, clearance, now)
        elif clearance is None:
            self.abort(now, reading.reason or "distance unavailable during approach")
        elif clearance < self.settings.pickup_floor_cm:
            self.abort(now, "ultrasonic clearance below the pickup window")
        elif target_cm is None or abs(error) > self.settings.realign_tolerance:
            self._realign(now)
        else:
            self._approach(error, reading, target_cm, clearance, now)

    def _align(
        self, error: float, reading: DistanceReading, target_cm: float | None,
        clearance: float | None, now: float,
    ) -> None:
        if abs(error) > self.settings.center_tolerance:
            self._aligned_at = None
            self._turn_or_hold(self._angular(error), clearance, now)
            return
        self._link.set_motion(STOP, now)
        if not self._link.stopped:
            return
        if self._aligned_at is None:
            self._aligned_at = now
        sample = reading.sample
        if target_cm is not None and sample is not None and sample.measured_at >= self._aligned_at + self.settings.settle_s:
            self._transition(State.APPROACH, now)

    def _approach(
        self, error: float, reading: DistanceReading, target_cm: float, clearance: float, now: float,
    ) -> None:
        limits = self.settings
        if clearance <= limits.pickup_cm:
            self._braking = True
        elif clearance >= limits.pickup_cm + limits.resume_margin_cm:
            self._reset_braking()

        if self._braking:
            if abs(error) > limits.center_tolerance:
                self._realign(now)
                return
            self._link.set_motion(STOP, now)
            self._confirm_pickup(reading, target_cm, now)
            return

        angular = self._angular(error) if abs(error) > limits.center_tolerance / 3 else 0.0
        linear = min(limits.max_linear, limits.approach_gain_per_cm * (clearance - limits.pickup_floor_cm))
        if angular and self._link.transport.supports_combined_velocity:
            command = Command(CommandKind.VELOCITY, linear, angular)
        elif abs(error) > limits.center_tolerance:
            command = Command(CommandKind.TURN, angular=angular)
        else:
            command = Command(CommandKind.MOVE, linear=linear)
        self._link.set_motion(command, now)

    def _confirm_pickup(self, reading: DistanceReading, target_cm: float, now: float) -> None:
        if not self._link.stopped:
            return
        if self._stopped_at is None:
            self._stopped_at = now
        if not self._within_pickup_window(reading, target_cm):
            self._confirmations = 0
            return
        sample = reading.sample
        if (sample is not None and sample.measured_at > self._confirmed_at
                and sample.measured_at >= self._stopped_at + self.settings.settle_s):
            self._confirmations += 1
            self._confirmed_at = sample.measured_at
        if self._confirmations >= self.settings.pickup_samples:
            self._transition(State.PICKUP, now)

    def _pickup(self, target: Detection | None, reading: DistanceReading, target_cm: float | None, now: float) -> None:
        if now - self._state_since >= self.settings.pickup_timeout_s:
            self.abort(now, "arm sequence timed out")
            return
        if self._action_sequence is not None:
            if not self._link.action_done(self._action_sequence):
                return
            self._action_index += 1
            if self._action_index == len(self.settings.pickup_sequence):
                self._transition(State.VERIFY, now)
                return
            self._action_sequence = None
        if self._action_index == 0 and (
            target is None or abs(target.cx - self._ranger.aim_x) > self.settings.center_tolerance
            or not self._within_pickup_window(reading, target_cm)
        ):
            self.abort(now, "target or range changed before the arm sequence")
            return

        sequence = self._link.start_action(self.settings.pickup_sequence[self._action_index], now)
        if sequence is not None:
            self._action_sequence = sequence
            self._action_sent_at = now

    def _verify(self, now: float) -> None:
        feedback = self._link.grip
        timeout = self.settings.verify_timeout_s
        if (feedback is not None and feedback.sequence == self._action_sequence
                and feedback.measured_at >= self._action_sent_at and is_fresh(feedback.measured_at, now, timeout)):
            if not feedback.holding:
                self.abort(now, "gripper sensor reported no object")
            else:
                self.result = PickupResult.VERIFIED
                self._transition(State.COMPLETE, now)
        elif now - self._state_since >= timeout:
            self.result = PickupResult.UNVERIFIED
            self.reason = "no fresh gripper confirmation"
            self._transition(State.COMPLETE, now)

    def _turn_or_hold(self, speed: float, clearance: float | None, now: float) -> None:
        safe = clearance is not None and clearance > self.settings.pickup_floor_cm
        optional = clearance is None and not self.settings.require_range_for_turn
        if speed and (safe or optional):
            self._link.set_motion(Command(CommandKind.TURN, angular=speed), now)
        else:
            self._link.set_motion(STOP, now)

    def _angular(self, error: float) -> float:
        return copysign(min(self.settings.max_angular, abs(error) * self.settings.turn_gain), error)

    def _within_pickup_window(self, reading: DistanceReading, target_cm: float | None) -> bool:
        raw = reading.raw_cm
        low, high = self.settings.pickup_floor_cm, self.settings.pickup_cm
        return raw is not None and target_cm is not None and low <= raw <= high and low <= target_cm <= high

    @staticmethod
    def _clearance(reading: DistanceReading) -> float | None:
        raw = reading.raw_cm
        if raw is not None and reading.filtered_cm is not None:
            return min(raw, reading.filtered_cm)
        return None

    def _reset_braking(self) -> None:
        self._braking = False
        self._stopped_at = None
        self._confirmations = 0
        self._confirmed_at = float("-inf")
