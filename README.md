# Autonomous object pickup

`robot_pickup` detects colored objects, tracks them across frames, and runs one pickup attempt. With no camera arguments, it uses a synthetic camera, range sensor, and Arduino. The Firebase dashboard has its own YOLO entry point, `vision_worker.py`.

## Install and run

Use Python 3.10 or newer in a virtual environment. Run the commands from the repository root.

```sh
python -m pip install -e .
python -m robot_pickup --simulate
python -m unittest discover -s tests -v
```

The simulation draws a colored object, updates its position and apparent size based on mock motion, and produces gripper feedback from simulated contact. Add `--display` for a live OpenCV window or `--verbose` to log command transmissions. The window shows object IDs, centers, bounding boxes, the selected target and its associated distance, the current state, and the last base command. Use `Q`, `Esc`, or `Ctrl+C` to exit.

| Camera input | Command |
| --- | --- |
| Local webcam | `python -m robot_pickup --source 0 --display` |
| Recorded video | `python -m robot_pickup --source footage.mp4 --display` |
| ESP32-CAM | `python -m robot_pickup --source http://192.168.1.50:81/stream --display` |

For the ESP32-CAM, pass the board's actual HTTP/MJPEG URL to `--source`. Espressif's CameraWebServer example registers `/stream` on the stream server, one port above the main HTTP server. [Espressif CameraWebServer source](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/app_httpd.cpp)

Camera sessions start in `IDLE`. Add `--activate` to exercise the controller with the mock Arduino. Add `--config config/pickup.example.json` to load the example settings, which detect red and green objects and select red. An empty `target.labels` list allows any detected label. `target.strategy` accepts `centered` or `largest`. HSV detections have no confidence score. Configure hue as `0–179` and saturation and value as `0–255` under `perception.colors`. The two red ranges handle hue wraparound.

To detect a black or wrapped eraser, install the optional YOLO extra with `python -m pip install -e '.[yolo]'` and use the preset for the bundled `best.pt` model.

```sh
python -m robot_pickup --source 0 --display --config config/eraser.example.json
```

For other YOLO models, set `perception.yolo_weights` and `target.labels` in your configuration. Preview on a camera or video before using a model for control.

## Range input and calibration

The serial adapter shares a port for command exchange and range telemetry. `--source 0 --serial-port COM3 --distance serial --display` opens a telemetry-only session with a mock command link. In the preview, `beam` shows the latest valid raw echo. Without a port or network endpoint, a camera session has unknown range.

For network telemetry, use `--distance http --distance-url http://192.168.1.51/distance`. The endpoint returns JSON in the following shape. Set `measured_at` to the actual Unix timestamp in seconds at acquisition, using a clock synchronized with the Python host. A repeated timestamp does not refresh the reading. Use `"valid": false` and `"distance_cm": null` for a timeout or no echo.

```json
{"distance_cm": 24.8, "valid": true, "measured_at": 1795424400.25}
```

1. In preview, tune `perception.colors`, `min_area_px`, and `max_area_fraction` with the target under expected lighting. Confirm that target IDs persist over normal motion.
2. Measure the ultrasonic beam with known objects at several positions in the operating range. Set `beam.center_x` and `center_y` to its projected aim point and `half_width` and `half_height` to cover the plausible echoes in the camera image. Image centers use `[-1, 1]` with positive x to the right and positive y down. The overlay crosshair shows the configured aim point. Set `beam.calibrated` to `true` after checking the mount and repeatability.
3. Compare range readings to measured distances. Tune `distance.min_cm`, `max_cm`, `max_age_s`, `median_window`, and `beam.max_skew_s` for the observed range, telemetry rate, and camera latency. An invalid sample clears the median window. The controller also uses the latest raw measurement to stop for a sudden short echo.
4. Measure the range from the sensor face where the stopped gripper can reliably reach the object. Calibrate `control.pickup_cm` and `pickup_window_cm` so the valid interval `[pickup_cm - pickup_window_cm, pickup_cm]` is inside this reach. Tune speed limits, braking margin, settling time, and the number of distinct confirming samples. Test alignment and stopping at low speeds before trying the arm sequence.

Only the selected, visible object can receive a distance estimate. It must cover the calibrated aim point, have a recent coherent sample, and have no other detected bounding box intersect the projected beam. Use a clear, controlled workspace. Invisible clutter, wall and floor echoes, and camera or telemetry buffering can make an association wrong. The tracker can switch IDs when similar objects overlap or cross.

## Arduino integration

Install the optional serial dependency with `python -m pip install -e '.[serial]'`. Once the Arduino team implements or maps the protocol and has tested its independent watchdog, run a hardware session with a calibrated copy of the configuration.

```sh
python -m robot_pickup --source http://192.168.1.50:81/stream --config config/pickup.local.json --hardware --activate --serial-port COM3 --distance serial --display
```

`--hardware` requires `--activate`, a live camera, a serial port, a real range input, and `beam.calibrated`. The controller starts with an acknowledged `STOP`. Search is stationary with the default `control.search_angular` of `0`. Set `serial.combined_velocity` to `true` after the firmware supports simultaneous linear and angular motion. `RECOVER` attempts a `STOP` and ends the run. Resolve the fault before starting a new attempt.

An acknowledgment confirms acceptance of a command. Each arm step must also report completion. A fresh gripper reading correlated with the last arm step determines the pickup result. Exit code `0` means verified pickup or normal preview exit, `2` means an error or failed pickup, `3` means an unverified pickup, and `130` means the operator ended an active run.

The Arduino team needs to agree on transport and baud rate, velocity units and signs or PWM mapping, arm completion feedback and timings, the range telemetry format and sample rate, gripper feedback, and watchdog behavior. See the [Arduino protocol proposal](docs/arduino-protocol.md) for the contract used by the Python adapter.
