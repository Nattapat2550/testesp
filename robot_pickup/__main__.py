import argparse
import logging
from contextlib import ExitStack
from importlib.util import find_spec
from ipaddress import IPv4Address
from pathlib import Path
from threading import Event
from time import monotonic

import cv2

from robot_pickup.communication.commands import CommandLink, MockArduino
from robot_pickup.communication.serial_adapter import SerialArduino
from robot_pickup.config import Config, load_config
from robot_pickup.control.controller import PickupController, PickupResult
from robot_pickup.display import DebugWindow
from robot_pickup.models import is_fresh
from robot_pickup.perception.camera import CameraSource, CameraVision
from robot_pickup.position_output import run_position_output
from robot_pickup.sensors.association import BeamAssociation
from robot_pickup.sensors.distance import DistanceProvider, FilteredDistance, HTTPDistance, NoDistance
from robot_pickup.simulation import Simulation

LOG = logging.getLogger(__name__)


def _result(controller: PickupController, mode: str) -> int:
    result = controller.result
    detail = f" ({controller.reason})" if controller.reason else ""
    LOG.info("%s pickup result: %s%s", mode, result.value, detail)
    return {PickupResult.VERIFIED: 0, PickupResult.FAILED: 2, PickupResult.UNVERIFIED: 3}[result]


def _run_simulation(config: Config, display: bool) -> int:
    demo = Simulation(config)
    window = DebugWindow() if display else None
    pace = Event()
    try:
        while True:
            started = monotonic()
            snapshot, status = demo.step()
            if window is not None and not window.show(snapshot, status, demo.beam):
                return 130
            if demo.controller.terminal:
                return _result(demo.controller, "Simulated")
            if window is not None:
                pace.wait(max(0, started + demo.interval_s - monotonic()))
    finally:
        demo.stop()
        if window is not None:
            window.close()


def _run_camera(args: argparse.Namespace, config: Config, source: CameraSource, distance_kind: str) -> int:
    mode = "Hardware" if args.hardware else "Dry-run"
    LOG.info("%s camera session%s", mode, " (preview only)" if not args.activate else "")
    if config.perception.backend == "hsv":
        LOG.info("HSV detector colors: %s", ", ".join(color.label for color in config.perception.colors))
        detector_name = "HSV colors"
    else:
        LOG.info("YOLO detector weights: %s", config.perception.yolo_weights)
        detector_name = f"YOLO ({Path(config.perception.yolo_weights).name})"
    if distance_kind == "none":
        LOG.info("No range sensor configured. Preview will show detections and tracking IDs.")
    vision = CameraVision(source, config.camera, config.perception.pipeline())
    pace = Event()
    with ExitStack() as stack:
        serial = None
        if args.serial_port:
            wire = config.serial
            serial = SerialArduino.open(args.serial_port, wire.baudrate, wire.write_timeout_s, wire.combined_velocity)
            stack.callback(serial.close)
        link = CommandLink(serial if args.hardware else MockArduino(
            supports_combined_velocity=config.serial.combined_velocity,
        ), config.link)
        serial_ready = monotonic() + config.serial.startup_s if serial is not None else 0.0

        if distance_kind == "serial":
            provider: DistanceProvider = serial
        elif distance_kind == "http":
            url = config.distance.http_url
            if url is None:
                raise ValueError("set distance.http_url in the project configuration to use HTTP distance")
            http = HTTPDistance(url, config.runtime.http_timeout_s, config.runtime.http_poll_s)
            http.start()
            stack.callback(http.close)
            provider = http
        else:
            provider = NoDistance()

        controller = PickupController(link, FilteredDistance(provider, config.distance), BeamAssociation(config.beam),
                                      config.control, config.target)
        vision.start()
        stack.callback(vision.close)
        window = DebugWindow(f"Robot pickup - {detector_name}") if args.display else None
        if window is not None:
            stack.callback(window.close)
        stack.callback(lambda: controller.abort(monotonic(), "run ended by operator"))
        startup_deadline = monotonic() + config.camera.startup_timeout_s
        armed = False
        visible_ids: set[int | None] | None = None
        while True:
            snapshot = vision.latest
            now = monotonic()
            if vision.finished.is_set():
                if vision.error:
                    LOG.error("%s", vision.error)
                    controller.abort(now, vision.error)
                    return _result(controller, mode) if armed else 2
                if not armed:
                    return 0
                controller.abort(now, "recorded video ended")
                return _result(controller, mode)
            if snapshot is None and now >= startup_deadline:
                LOG.error("No processed camera frame before the startup timeout")
                return 2

            scene = snapshot.scene if snapshot is not None else None
            if args.activate and not armed:
                fresh_frame = scene is not None and is_fresh(scene.observed_at, now, config.control.frame_timeout_s)
                if fresh_frame and now >= serial_ready:
                    controller.activate(now)
                    armed = True
                elif now >= max(startup_deadline, serial_ready + config.control.frame_timeout_s):
                    LOG.error("No fresh camera frame for activation")
                    return 2
            if serial is not None and not args.hardware:
                serial.poll(now)
            status = controller.tick(scene, now)
            if scene is not None:
                ids = {obj.object_id for obj in scene.objects}
                if ids != visible_ids:
                    LOG.info("Visible objects: %s", ", ".join(f"{obj.label} #{obj.object_id}" for obj in scene.objects) or "none")
                    visible_ids = ids
            if window is not None and snapshot is not None and not window.show(snapshot, status, config.beam):
                return 130 if armed else 0
            if controller.terminal:
                return _result(controller, mode)
            pace.wait(max(0, now + 1 / config.runtime.tick_hz - monotonic()))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Detect and pick up a configured object using a camera and range sensor.")
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument("--simulate", action="store_true", help="run the built-in simulation (default without a source)")
    inputs.add_argument("--source", help="webcam index, HTTP/MJPEG URL, or local video file")
    parser.add_argument("--config", type=Path, help="JSON settings file inside config/ (see config/pickup.example.json)")
    parser.add_argument("--display", action="store_true", help="show the OpenCV preview; press Q or Esc to exit")
    parser.add_argument("--position-only", action="store_true", help="output target coordinates as JSON over Wi-Fi or USB")
    parser.add_argument("--activate", action="store_true", help="start the controller once a camera frame arrives")
    parser.add_argument("--hardware", action="store_true", help="send commands to a serial Arduino instead of the mock")
    links = parser.add_mutually_exclusive_group()
    links.add_argument("--serial-port", help="Arduino port, e.g. COM3 or /dev/ttyACM0")
    links.add_argument("--target-ip", type=IPv4Address, help="Wi-Fi IPv4 address of the controller ESP32 in position-only mode")
    parser.add_argument("--distance", choices=("serial", "http", "none"), help="range telemetry source")
    parser.add_argument("--distance-url", help="use HTTP distance with the URL already configured in distance.http_url")
    parser.add_argument("--verbose", action="store_true", help="include command transmissions in logs")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s %(message)s")

    try:
        if args.target_ip is not None and not args.position_only:
            parser.error("--target-ip requires --position-only")
        if args.target_ip is not None and (args.target_ip.is_unspecified or args.target_ip.is_multicast
                                          or args.target_ip == IPv4Address("255.255.255.255")):
            parser.error("--target-ip must be the controller ESP32's unicast IPv4 address")
        if args.position_only and (args.source is None or args.activate or args.hardware
                                   or args.distance or args.distance_url is not None):
            parser.error("use --position-only with --source and optional --target-ip or --serial-port, without activation or distance options")
        config_path = args.config
        if (config_path is None and args.source is not None and not (args.activate or args.hardware)
                and (args.position_only or args.source.isdecimal())):
            eraser_preset = Path("config/eraser.example.json")
            if eraser_preset.is_file() and Path("best.pt").is_file() and find_spec("ultralytics") is not None:
                config_path = eraser_preset
        config = load_config(config_path)
        if args.source is None:
            if args.hardware or args.serial_port or args.distance or args.distance_url is not None:
                parser.error("camera source is required for hardware and external distance options")
            return _run_simulation(config, args.display)

        source = CameraSource.parse(args.source)
        if args.position_only:
            return run_position_output(config, source, args.display, args.serial_port, target_ip=args.target_ip)
        distance_kind = args.distance or ("http" if args.distance_url is not None else "serial" if args.serial_port else "none")
        if distance_kind == "serial" and not args.serial_port:
            parser.error("serial distance requires --serial-port")
        if args.distance_url is not None and (distance_kind != "http" or args.distance_url != config.distance.http_url):
            parser.error("--distance-url must match distance.http_url in your project configuration")
        if distance_kind == "http" and config.distance.http_url is None:
            parser.error("set distance.http_url in your project configuration to use HTTP distance")
        if args.serial_port and not args.hardware and distance_kind != "serial":
            parser.error("--serial-port requires --hardware or serial distance telemetry")
        if args.hardware:
            if not (args.activate and args.serial_port and source.is_live and distance_kind != "none"):
                parser.error("hardware requires --activate, --serial-port, a live camera, and real distance telemetry")
            if not config.beam.calibrated:
                parser.error("hardware requires beam.calibrated in the config after checking the camera/sensor geometry")
        return _run_camera(args, config, source, distance_kind)
    except KeyboardInterrupt:
        return 130
    except (ValueError, TypeError, KeyError, OSError, cv2.error) as exc:
        LOG.error("%s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
