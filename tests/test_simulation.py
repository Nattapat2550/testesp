import io
import unittest
from contextlib import redirect_stderr
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from robot_pickup.__main__ import main
from robot_pickup.communication.commands import ARM_COMMANDS, CommandKind
from robot_pickup.config import load_config
from robot_pickup.control.controller import PickupResult, State
from robot_pickup.display import draw_overlay
from robot_pickup.simulation import Simulation


class SimulationTests(unittest.TestCase):
    def test_example_config_runs_complete_detection_to_pickup_loop(self):
        config = load_config(Path("config/pickup.example.json"))
        config = replace(config, serial=replace(config.serial, combined_velocity=True))
        demo = Simulation(config)
        visited = {demo.controller.state}
        selected = set()
        overlay_checked = False
        for _ in range(500):
            snapshot, status = demo.step()
            visited.add(status.state)
            if status.target_visible:
                selected.add(status.target_id)
            if status.state is State.APPROACH and not overlay_checked:
                self.assertGreater(status.target_distance_cm, 0)
                original = snapshot.frame.copy()
                view = draw_overlay(snapshot, status, demo.beam)
                self.assertEqual(view.shape, original.shape)
                self.assertFalse(np.array_equal(view, original))
                np.testing.assert_array_equal(snapshot.frame, original)
                overlay_checked = True
            if demo.controller.terminal:
                break

        self.assertTrue(overlay_checked)
        self.assertEqual(status.result, PickupResult.VERIFIED)
        self.assertEqual(visited, set(State) - {State.SEARCH, State.RECOVER})
        self.assertEqual(len(selected), 1)
        kinds = [item.command.kind for item in demo.arduino.history]
        self.assertIn(CommandKind.TURN, kinds)
        self.assertIn(CommandKind.VELOCITY, kinds)
        self.assertEqual([kind for kind in kinds if kind in ARM_COMMANDS],
                         [CommandKind.LOWER, CommandKind.GRAB, CommandKind.LIFT])

    def test_hardware_flag_requires_activation_and_calibration_before_open(self):
        for flags in ([], ["--activate"]):
            with self.subTest(flags=flags), patch("robot_pickup.__main__.SerialArduino.open") as serial_open:
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    main(["--source", "0", "--serial-port", "COM3", "--hardware", *flags])
                self.assertEqual(error.exception.code, 2)
                serial_open.assert_not_called()

    def test_webcam_preview_uses_bundled_eraser_model_when_available(self):
        cases = (
            (object(), [], "yolo"),
            (None, [], "hsv"),
            (object(), ["--config", "config/pickup.example.json"], "hsv"),
            (object(), ["--activate"], "hsv"),
        )
        for yolo_available, flags, expected_backend in cases:
            with self.subTest(flags=flags, backend=expected_backend):
                with patch("robot_pickup.__main__.find_spec", return_value=yolo_available), \
                     patch("robot_pickup.__main__._run_camera", return_value=0) as run_camera:
                    self.assertEqual(main(["--source", "0", "--display", *flags]), 0)
                    self.assertEqual(run_camera.call_args.args[1].perception.backend, expected_backend)
