from dataclasses import replace

import cv2
import numpy as np

from robot_pickup.communication.commands import Command, CommandKind, CommandLink, MockArduino
from robot_pickup.config import Config
from robot_pickup.control.controller import ControlStatus, PickupController
from robot_pickup.perception.camera import VisionSnapshot
from robot_pickup.perception.detector import Frame, HSVRange
from robot_pickup.sensors.association import BeamAssociation
from robot_pickup.sensors.distance import FilteredDistance, SimulatedDistance


class SimulatedScene:
    """A colored object in a camera view that responds to drive and gripper commands."""

    width, height = 480, 360

    def __init__(self, config: Config, color: HSVRange) -> None:
        self._control = config.control
        self._beam = config.beam
        self._distance = config.control.pickup_cm + min(43.0, (config.distance.max_cm - config.control.pickup_cm) / 2)
        self._wall_distance = min(180.0, config.distance.max_cm)
        self._offset = 0.4 if self._beam.center_x <= 0 else -0.4
        self._lowered = False
        self._holding = False
        self._lifted = False
        hsv = np.array([[[int((lo + hi) / 2) for lo, hi in zip(color.low, color.high)]]], dtype=np.uint8)
        self._bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]

    def advance(self, motion: Command, dt: float) -> None:
        self._offset -= motion.angular * 1.7 * dt
        self._distance = max(2.0, self._distance - motion.linear * 70 * dt)

    def _half_size(self) -> int:
        return max(14, min(90, round(1150 / self._distance)))

    def frame(self) -> Frame:
        frame = np.full((self.height, self.width, 3), 28, dtype=np.uint8)
        if self._holding and self._lifted:
            return frame
        x = round((1 + self._beam.center_x + self._offset) * self.width / 2)
        y = round((1 + self._beam.center_y) * self.height / 2)
        radius = self._half_size()
        x1, x2 = max(0, x - radius), min(self.width, x + radius)
        y1, y2 = max(0, y - radius), min(self.height, y + radius)
        if x1 < x2 and y1 < y2:
            frame[y1:y2, x1:x2] = self._bgr
        return frame

    def measure(self) -> float:
        echo_from_target = abs(self._offset) * self.width / 2 < self._half_size() and not self._lifted
        return self._distance if echo_from_target else self._wall_distance

    def act(self, action: CommandKind) -> bool:
        if action is CommandKind.LOWER:
            self._lowered = True
            self._lifted = False
        elif action is CommandKind.GRAB:
            self._holding = (
                self._lowered and abs(self._offset) <= self._control.center_tolerance
                and self._control.pickup_floor_cm <= self._distance <= self._control.pickup_cm
            )
        elif action is CommandKind.LIFT:
            self._lifted = True
            self._lowered = False
        elif action is CommandKind.RELEASE:
            self._holding = False
        return self._holding


class Simulation:
    def __init__(self, config: Config) -> None:
        if config.perception.backend != "hsv":
            raise ValueError("the built-in simulation uses the HSV detector")
        color = next((color for color in config.perception.colors
                      if not config.target.labels or color.label in config.target.labels), None)
        if color is None:
            raise ValueError("simulation needs an HSV color allowed by the target policy")
        self.beam = replace(config.beam, calibrated=True)
        self.interval_s = 1 / config.runtime.tick_hz
        self._pipeline = config.perception.pipeline()
        self._scene = SimulatedScene(config, color.ranges[0])
        self.arduino = MockArduino(on_action=self._scene.act, supports_combined_velocity=config.serial.combined_velocity)
        distance = FilteredDistance(SimulatedDistance(self._scene.measure), config.distance)
        self.controller = PickupController(CommandLink(self.arduino, config.link), distance,
                                           BeamAssociation(self.beam), config.control, config.target)
        self._ticks = 0

    def step(self) -> tuple[VisionSnapshot, ControlStatus]:
        now = self._ticks * self.interval_s
        if not self._ticks:
            self.controller.activate(now)
        else:
            self._scene.advance(self.arduino.motion, self.interval_s)
        frame = self._scene.frame()
        scene = self._pipeline.process(frame, now)
        self._ticks += 1
        return VisionSnapshot(frame, scene), self.controller.tick(scene, now)

    def stop(self) -> None:
        self.controller.abort(self._ticks * self.interval_s, "simulation ended by operator")
