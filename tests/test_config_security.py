import io
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from robot_pickup.__main__ import main
from robot_pickup.config import Config, load_config
from robot_pickup.sensors.distance import DistanceSettings


class ConfigPathTests(unittest.TestCase):
    def setUp(self) -> None:
        project = TemporaryDirectory()
        self.addCleanup(project.cleanup)
        self.project = Path(project.name).resolve()
        self.config_dir = self.project / "config"
        self.config_dir.mkdir()
        self.config = self.config_dir / "sensor.json"
        self.config.write_text('{"target":{"labels":["red"]}}', encoding="utf-8")
        self.outside = self.project / "other.json"
        self.outside.write_text("{}", encoding="utf-8")
        cwd = patch.object(Path, "cwd", return_value=self.project)
        cwd.start()
        self.addCleanup(cwd.stop)

    def test_loads_relative_and_absolute_paths_in_config(self) -> None:
        for path in (Path("config/sensor.json"), self.config):
            with self.subTest(path=path):
                self.assertEqual(load_config(path).target.labels, ("red",))

    def test_rejects_paths_outside_config_and_parent_segments(self) -> None:
        for path in (self.outside, Path("other.json"), Path("config/../other.json")):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, "inside the project's config/"):
                load_config(path)

    def test_rejects_config_files_that_resolve_outside_config(self) -> None:
        resolve = Path.resolve

        def resolve_link(path: Path, strict: bool = False) -> Path:
            return self.outside if path == self.config else resolve(path, strict=strict)

        with patch.object(Path, "resolve", resolve_link), self.assertRaisesRegex(ValueError, "inside the project's config/"):
            load_config(self.config)


class DistanceEndpointCLITests(unittest.TestCase):
    url = "http://192.168.1.51/distance"

    def test_http_requires_an_endpoint_in_the_loaded_config(self) -> None:
        with patch("robot_pickup.__main__.load_config", return_value=Config()), \
                patch("robot_pickup.__main__._run_camera") as run, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main(["--source", "0", "--distance", "http"])
            self.assertEqual(error.exception.code, 2)
            run.assert_not_called()

    def test_cli_can_select_but_cannot_replace_the_configured_endpoint(self) -> None:
        config = Config(distance=DistanceSettings(http_url=self.url))
        with patch("robot_pickup.__main__.load_config", return_value=config), \
                patch("robot_pickup.__main__._run_camera", return_value=0) as run, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main(["--source", "0", "--distance-url", "http://192.168.1.52/distance"])
            self.assertEqual(error.exception.code, 2)
            run.assert_not_called()

            self.assertEqual(main(["--source", "0", "--distance", "http"]), 0)
            self.assertEqual(main(["--source", "0", "--distance-url", self.url]), 0)
            self.assertEqual(run.call_count, 2)
            for call in run.call_args_list:
                self.assertIs(call.args[1], config)
                self.assertEqual(call.args[3], "http")
