import io
import json
import unittest
from contextlib import redirect_stderr
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from robot_pickup.__main__ import main
from robot_pickup.control.controller import TargetPolicy
from robot_pickup.models import BoundingBox, Detection, Scene
from robot_pickup.position_output import PositionReporter, save_pickup_point


class PositionReporterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lines: list[str] = []
        self.reporter = PositionReporter(TargetPolicy(labels=("black-eraser",)), 0.4, self.lines.append)
        self.eraser = Detection("black-eraser", BoundingBox(320, 80, 480, 160), 0.25, -0.5, 0.04, 0.73, 1)

    def test_reports_only_selected_eraser_with_both_coordinate_systems(self) -> None:
        red = Detection("red", BoundingBox(300, 200, 340, 240), 0.0, -0.0833, 0.01, None, 2)
        self.assertEqual(self.reporter.update(Scene(0, 640, 480, (red, self.eraser)), 0.05), self.eraser)
        self.assertEqual(json.loads(self.lines[0]), {
            "type": "target", "found": True, "label": "black-eraser", "object_id": 1,
            "confidence": 0.73, "cx": 0.25, "cy": -0.5, "x_px": 400.0, "y_px": 120.0,
            "frame_width": 640, "frame_height": 480,
        })

    def test_limits_position_rate_and_never_resends_an_old_frame(self) -> None:
        scene = Scene(0, 640, 480, (self.eraser,))
        self.reporter.update(scene, 0.01)
        self.reporter.update(Scene(0.05, 640, 480, (self.eraser,)), 0.06)
        self.reporter.update(scene, 0.15)
        self.assertEqual(len(self.lines), 1)
        self.reporter.update(Scene(0.2, 640, 480, (self.eraser,)), 0.21)
        self.assertEqual(len(self.lines), 2)

    def test_clears_stale_positions_and_on_shutdown(self) -> None:
        scene = Scene(0, 640, 480, (self.eraser,))
        self.reporter.update(scene, 0.05)
        self.assertIsNone(self.reporter.update(scene, 0.41))
        self.reporter.update(None, 0.5)
        self.reporter.update(Scene(0.55, 640, 480, (self.eraser,)), 0.55)
        self.reporter.clear()
        self.reporter.clear()
        self.assertEqual([json.loads(line)["found"] for line in self.lines], [True, False, True, False])

    def test_position_cli_opens_position_output_and_rejects_motor_flags(self) -> None:
        with patch("robot_pickup.__main__.run_position_output", return_value=0) as run:
            self.assertEqual(main(["--source", "0", "--position-only", "--serial-port", "COM3"]), 0)
            self.assertEqual(run.call_args.args[3], "COM3")
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(["--source", "0", "--position-only", "--hardware", "--activate"])
            self.assertEqual(error.exception.code, 2)
            run.assert_called_once()

    def test_saves_a_fresh_pickup_point_and_preserves_it_when_target_is_missing_or_stale(self) -> None:
        scene = Scene(1.0, 640, 480, (self.eraser,))
        with TemporaryDirectory() as directory:
            path = Path(directory) / "pickup-point.json"
            with patch("robot_pickup.position_output.PICKUP_POINT_PATH", path):
                with patch("robot_pickup.position_output.monotonic", return_value=1.1):
                    save_pickup_point(scene, self.eraser, 0.4)
                    save_pickup_point(scene, None, 0.4)
                with patch("robot_pickup.position_output.monotonic", return_value=1.5):
                    moved = replace(self.eraser, cx=0.5)
                    save_pickup_point(replace(scene, objects=(moved,)), moved, 0.4)
                self.assertEqual(json.loads(path.read_text(encoding="utf-8")), {
                    "label": "black-eraser", "pickup_cx": 0.25, "pickup_cy": -0.5,
                    "frame_width": 640, "frame_height": 480,
                })
