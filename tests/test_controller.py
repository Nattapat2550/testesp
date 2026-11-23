import unittest
from dataclasses import replace

from robot_pickup.communication.commands import ARM_COMMANDS, STOP, Ack, CommandKind, CommandLink, LinkSettings, MockArduino
from robot_pickup.control.controller import ControllerSettings, PickupController, PickupResult, State, TargetPolicy
from robot_pickup.models import BoundingBox, Detection, DistanceSample, Scene
from robot_pickup.sensors.association import BeamAssociation, BeamSettings
from robot_pickup.sensors.distance import DistanceSettings, FilteredDistance


class TestRange:
    sample: DistanceSample | None = None

    def latest(self, now: float) -> DistanceSample | None:
        return self.sample


def object_at(cx=0.0, object_id=1, label="red", half_width=10, confidence=None):
    center = round((1 + cx) * 50)
    box = BoundingBox(center - half_width, 38, center + half_width, 62)
    return replace(Detection.from_box(label, box, 100, 100, confidence), object_id=object_id)


class Rig:
    def __init__(self, arduino=None, **settings):
        self.now = 0.0
        self.arduino = MockArduino() if arduino is None else arduino
        self.range = TestRange()
        self.controller = PickupController(
            CommandLink(self.arduino, LinkSettings()),
            FilteredDistance(self.range, DistanceSettings(median_window=1)),
            BeamAssociation(BeamSettings(calibrated=True)),
            ControllerSettings(**settings), TargetPolicy(labels=("red",)),
        )

    def arm_kinds(self):
        return [entry.command.kind for entry in self.arduino.history if entry.command.kind in ARM_COMMANDS]

    def step(self, cm=25.0, objects=(object_at(),), update_range=True, observed_at=None):
        self.now = round(self.now + 0.05, 8)
        if update_range:
            self.range.sample = DistanceSample(cm, self.now, cm is not None)
        scene = Scene(self.now if observed_at is None else observed_at, 100, 100, objects)
        return self.controller.tick(scene, self.now)

    def reach(self, state, cm=25.0):
        for _ in range(160):
            status = self.step(cm=cm)
            if status.state is state or self.controller.terminal:
                return status
        raise AssertionError(f"{state.value} was never reached")


class SelectTests(unittest.TestCase):
    def test_target_policy_applies_label_confidence_and_sort_order(self):
        small = object_at(cx=0.0, object_id=1, half_width=5)
        large = object_at(cx=0.2, object_id=2, half_width=14, confidence=0.8)
        weak = object_at(cx=0.0, object_id=3, half_width=20, confidence=0.2)
        green = object_at(cx=0.0, object_id=4, label="green", half_width=20)
        scene = Scene(0.0, 100, 100, (small, large, weak, green))
        self.assertEqual(TargetPolicy(("red",), "centered").select(scene, 0.0), small)
        self.assertEqual(TargetPolicy(("red",), "largest").select(scene, 0.0), large)


class ControllerTests(unittest.TestCase):
    def moving_rig(self, arduino=None, **settings):
        rig = Rig(arduino, **settings)
        rig.controller.activate(rig.now)
        self.assertEqual(rig.reach(State.APPROACH).state, State.APPROACH)
        self.assertEqual(rig.step().motion.kind, CommandKind.MOVE)
        return rig

    def test_idle_issues_no_commands(self):
        rig = Rig(search_angular=0.25)
        for _ in range(20):
            self.assertEqual(rig.step().state, State.IDLE)
        self.assertEqual(rig.arduino.history, [])

    def test_search_turns_with_clearance_and_stops_on_range_loss(self):
        rig = Rig(search_angular=0.25, search_timeout_s=0.30)
        rig.controller.activate(rig.now)
        status = rig.step(objects=())
        self.assertEqual(status.state, State.SEARCH)
        self.assertEqual(status.motion.kind, CommandKind.TURN)
        self.assertEqual(rig.step(cm=None, objects=()).motion, STOP)
        for _ in range(5):
            status = rig.step(cm=None, objects=())
        self.assertEqual(status.state, State.RECOVER)
        self.assertIn("search timed out", status.reason)

    def test_loss_stops_immediately_and_timeout_latches_recovery(self):
        rig = self.moving_rig(target_timeout_s=0.30)
        status = rig.step(objects=())
        self.assertEqual(status.state, State.ALIGN)
        self.assertEqual(status.motion, STOP)
        self.assertFalse(status.target_visible)
        self.assertIsNone(status.target_distance_cm)
        for _ in range(8):
            status = rig.step(objects=())
        self.assertEqual(status.state, State.RECOVER)
        self.assertIn("target was lost", status.reason)
        self.assertEqual(rig.arm_kinds(), [])

    def test_invalid_range_aborts_approach(self):
        rig = self.moving_rig()
        status = rig.step(cm=None)
        self.assertEqual(status.state, State.RECOVER)
        self.assertEqual(rig.arduino.motion, STOP)
        self.assertEqual(rig.arm_kinds(), [])

    def test_stale_range_stops_approach_and_prevents_restart(self):
        rig = self.moving_rig()
        for _ in range(9):
            status = rig.step(update_range=False)
        self.assertEqual(status.state, State.ALIGN)
        self.assertEqual(rig.arduino.motion, STOP)
        self.assertIsNone(status.target_distance_cm)
        self.assertEqual(rig.arm_kinds(), [])

    def test_camera_stall_stops_even_if_range_keeps_updating(self):
        rig = self.moving_rig()
        frozen = rig.now
        for _ in range(10):
            status = rig.step(observed_at=frozen)
        self.assertEqual(status.state, State.RECOVER)
        self.assertIn("camera frame", status.reason)
        self.assertEqual(rig.arduino.motion, STOP)

    def test_beam_occlusion_holds_base_stopped_until_motion_timeout(self):
        rig = self.moving_rig(motion_timeout_s=0.8)
        blocker = object_at(cx=0.1, object_id=2, label="green")
        for _ in range(20):
            status = rig.step(objects=(object_at(), blocker))
            self.assertEqual(rig.arduino.motion, STOP)
            self.assertIsNone(status.target_distance_cm)
        self.assertEqual(status.state, State.RECOVER)
        self.assertIn("timed out", status.reason)

    def test_steering_uses_supported_command_and_realigns_outside_larger_gate(self):
        for combined, expected in ((True, CommandKind.VELOCITY), (False, CommandKind.TURN)):
            with self.subTest(combined=combined):
                rig = self.moving_rig(MockArduino(supports_combined_velocity=combined), max_linear=0.15, max_angular=0.10)
                status = rig.step(objects=(object_at(cx=0.08),))
                self.assertEqual(status.state, State.APPROACH)
                self.assertEqual(status.motion.kind, expected)
                self.assertLessEqual(status.motion.linear, 0.15)
                self.assertEqual(status.motion.angular, 0.10)
                status = rig.step(objects=(object_at(cx=0.40),))
                self.assertEqual(status.state, State.ALIGN)
                self.assertEqual(status.motion, STOP)

    def test_pickup_checks_fresh_range_again_before_starting_arm(self):
        rig = Rig()
        rig.controller.activate(rig.now)
        self.assertEqual(rig.reach(State.PICKUP, cm=11.0).state, State.PICKUP)
        status = rig.step(cm=None)
        self.assertEqual(status.state, State.RECOVER)
        self.assertEqual(rig.arm_kinds(), [])

    def test_repeated_range_sample_cannot_supply_two_pickup_confirmations(self):
        rig = Rig(pickup_samples=2, settle_s=0.07)
        rig.controller.activate(rig.now)
        self.assertEqual(rig.reach(State.APPROACH, cm=11.0).state, State.APPROACH)
        for _ in range(3):
            self.assertEqual(rig.step(cm=11.0).state, State.APPROACH)
        for _ in range(3):
            status = rig.step(update_range=False)
        self.assertEqual(status.state, State.APPROACH)
        self.assertEqual(status.motion, STOP)
        self.assertEqual(rig.arm_kinds(), [])
        self.assertEqual(rig.step(cm=11.0).state, State.PICKUP)

    def test_one_arm_transmission_per_step_and_feedback_determines_result(self):
        cases = ((True, State.COMPLETE, PickupResult.VERIFIED), (None, State.COMPLETE, PickupResult.UNVERIFIED),
                 (False, State.RECOVER, PickupResult.FAILED))
        for feedback, terminal, result in cases:
            with self.subTest(feedback=feedback):
                rig = Rig(MockArduino(on_action=lambda kind: feedback), verify_timeout_s=0.25)
                rig.controller.activate(rig.now)
                status = rig.reach(terminal, cm=11.0)
                self.assertEqual(status.state, terminal)
                self.assertEqual(status.result, result)
                self.assertEqual(rig.arm_kinds(), [CommandKind.LOWER, CommandKind.GRAB, CommandKind.LIFT])
                self.assertEqual(rig.arduino.motion, STOP)

    def test_arm_completion_timeout_stops_and_does_not_retry(self):
        rig = Rig(MockArduino(action_time_s=100), pickup_timeout_s=0.25)
        rig.controller.activate(rig.now)
        status = rig.reach(State.RECOVER, cm=11.0)
        self.assertIn("arm sequence timed out", status.reason)
        self.assertEqual(rig.arm_kinds(), [CommandKind.LOWER])
        self.assertEqual(rig.arduino.history[-1].command, STOP)

    def test_missing_move_ack_attempts_emergency_stop(self):
        class DropMoveAck(MockArduino):
            def poll(self, now):
                move_ids = {item.sequence for item in self.history if item.command.kind is CommandKind.MOVE}
                return [event for event in super().poll(now)
                        if not isinstance(event, Ack) or event.sequence not in move_ids]

        rig = self.moving_rig(DropMoveAck())
        status = rig.reach(State.RECOVER)
        self.assertEqual(status.state, State.RECOVER)
        self.assertIn("ACK timeout for MOVE", status.reason)
        self.assertEqual(rig.arduino.history[-1].command, STOP)
