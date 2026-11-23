# Autonomous object pickup

`robot_pickup` detects colored objects, tracks them across frames, and runs one pickup attempt. With no camera arguments, it uses a synthetic camera, range sensor, and Arduino. The Firebase dashboard has its own YOLO entry point, `vision_worker.py`.

## Install and run

Use Python 3.10 or newer. Run these commands from the repository root to install the locked dependencies and run the simulation and tests.

```sh
uv sync --locked
uv run --locked python -m robot_pickup --simulate
uv run --locked python -m unittest discover -s tests -v
```

Include `--extra yolo` for eraser detection with `best.pt` and `--extra serial` for Arduino USB communication when syncing. Activate `.venv` for the `python` commands below. [uv lock and sync guide](https://docs.astral.sh/uv/concepts/projects/sync/)

The simulation draws a colored object, updates its position and apparent size based on mock motion, and produces gripper feedback from simulated contact. Add `--display` for a live OpenCV window or `--verbose` to log command transmissions. The window shows object IDs, centers, bounding boxes, the selected target and its associated distance, the current state, and the last base command. Use `Q`, `Esc`, or `Ctrl+C` to exit.

| Camera input | Command |
| --- | --- |
| Local webcam | `python -m robot_pickup --source 0 --display` |
| Recorded video | `python -m robot_pickup --source footage.mp4 --display` |
| ESP32-CAM | `python -m robot_pickup --source http://192.168.1.50:81/stream --display` |

For the ESP32-CAM, pass the board's actual HTTP/MJPEG URL to `--source`. Espressif's CameraWebServer example registers `/stream` on the stream server, one port above the main HTTP server. [Espressif CameraWebServer source](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/app_httpd.cpp)

Camera sessions start in `IDLE`. Add `--activate` to exercise the controller with the mock Arduino. Add `--config config/pickup.example.json` to load the example settings, which detect red and green objects and select red. An empty `target.labels` list allows any detected label. `target.strategy` accepts `centered` or `largest`. HSV detections have no confidence score. Configure hue as `0–179` and saturation and value as `0–255` under `perception.colors`. The two red ranges handle hue wraparound.

With the optional YOLO extra installed and `best.pt` present, a plain webcam preview from the repository root uses the eraser preset. The window title shows the active detector. You can also load the preset explicitly for a webcam, recorded video, or ESP32-CAM stream.

```sh
python -m robot_pickup --source 0 --display --config config/eraser.example.json
```

For other YOLO models, set `perception.yolo_weights` and `target.labels` in your configuration. Preview on a camera or video before using a model for control.

## Send the eraser position to Arduino

Use position-only mode when the Arduino firmware will decide how to move. With YOLO installed and `best.pt` in the project root, this mode selects the eraser preset by default. Run it from the project folder.

```powershell
.\.venv\Scripts\python.exe -m robot_pickup --source 0 --position-only --display
```

Replace `0` with the ESP32-CAM MJPEG URL to use the separate camera. Add `--serial-port COM3` with the ESP32 controller's actual port to send the same lines over USB at **115200 baud**. Close Serial Monitor while Python has the port open.

Each line reports the selected eraser's center. `cx` and `cy` run from −1 to +1. The image center is `(0, 0)`, x increases to the right, and y increases downward. `x_px` and `y_px` give the same position in pixels. See the [target position message format](docs/arduino-protocol.md#target-position-output) for the Arduino implementation.

Open the [ESP32 target receiver example](arduino/esp32_target_receiver/esp32_target_receiver.ino) in Arduino IDE with ArduinoJson 7 installed. It prints the next movement decision, including `STOP` when data is lost. See the [sample Python and ESP32 output](docs/arduino-protocol.md#example-output).

### Include the receiver in your main Arduino sketch

Copy [target_receiver.h](arduino/esp32_target_receiver/target_receiver.h) into the same folder as your main `.ino` file. Install ArduinoJson 7 in Arduino IDE. Add the include and receiver near the top of your `.ino`.

```cpp
#include "target_receiver.h"
TargetReceiver eraserReceiver(Serial);
```

Use `Serial.begin(115200);` once in your existing `setup()`. Then add this to your existing `loop()` before making a movement decision.

```cpp
eraserReceiver.poll();

TargetPosition eraser;
if (eraserReceiver.latest(eraser)) {
  // Use eraser.cx and eraser.cy in your existing movement logic.
} else {
  // Call your motor driver's stop function.
}
```

`latest()` returns false when Python reports target loss or 500 ms pass without a fresh position. Your sketch can still write ultrasonic readings to the same port. If the main sketch already reads complete serial lines, pass each line to `eraserReceiver.acceptLine(line)` from that reader and omit the separate `poll()` call.

### Teach the pickup spot

1. Fix the ESP32-CAM to the chassis on its centerline and aim it at the floor in front of the gripper. Keep the car parked with motor power off during setup.
2. Put the eraser where the gripper can reach it from a repeatable arm pose. Mark that spot on the floor and put the arm in its driving pose so the camera can see the eraser.
3. Run the position preview above with the mounted camera's MJPEG URL. Once it shows the eraser, focus the preview and press **P**. Python saves `pickup_cx` and `pickup_cy` to `pickup-point.json` in the project folder. Pressing **P** on a fresh detection replaces the saved point.
4. Give `pickup-point.json` and the target message format to the Arduino teammate. The firmware can compare each live `cx` and `cy` with the saved values, adjust the base, and stop inside a tolerance measured on the robot before running the arm's lower, close, and lift sequence. Check the correction directions on the assembled robot and relearn the pickup spot after changing the camera mount or image framing.

## Test the HC-SR04 with a camera

With USB unplugged and motor power disconnected, wire a standard 5 V HC-SR04 to your bench board. The ESP32 pin mapping is for a classic ESP32 DevKit or ESP32-WROOM-32 development board. Use the 5 V rail exposed by your board while powered over USB. The HC-SR04 `ECHO` output is 5 V and requires a divider before it reaches the ESP32 input. [ESP32 DevKitC pinout](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html) · [HC-SR04 pinout and logic levels](https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/pinouts)

| HC-SR04 pin | ESP32 DevKit | 5 V Uno R3 or classic Nano |
| --- | --- | --- |
| `VCC` | `5V` | `5V` |
| `TRIG` | `GPIO25` | `D7` |
| `ECHO` | `GPIO26` through a voltage divider | `D8` |
| `GND` | `GND` | `GND` |

For the ESP32, connect `ECHO` to `GPIO26` through a **2.2 kΩ** resistor. Connect **3.3 kΩ** from `GPIO26` to `GND`. This divides a 5 V echo to about 3.0 V at the GPIO. The sketch belongs on the ESP32 DevKit connected to the sensor. [ESP32 electrical specifications](https://documentation.espressif.com/esp32_datasheet_en.pdf)

### ESP32 setup in Arduino IDE

Add the following URL under **File → Preferences → Additional Boards Manager URLs**. In **Boards Manager**, install **esp32 by Espressif Systems**. Select **ESP32 Dev Module** for the classic DevKit or ESP32-WROOM-32 development board. [Espressif Arduino IDE installation guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)

```text
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

### Upload and test

1. Plug the board into the PC with a USB data cable. Open [the telemetry sketch](arduino/hcsr04_telemetry/hcsr04_telemetry.ino) in Arduino IDE, select your board and COM port, and click **Upload**. The sketch uses the board's Arduino core.
2. Open **Serial Monitor** at **115200 baud**. Aim the sensor at a flat object about 20 cm away and look for lines with `"type":"distance"`, `"valid":true`, and a distance near 20. The sketch reports an invalid sample when the echo times out.
3. Close Serial Monitor to free the COM port for Python. Run the webcam command below from the project root, substituting the sensor board's port shown in Arduino IDE. [Arduino serial port troubleshooting](https://support.arduino.cc/hc/en-us/articles/4407830972050-Find-and-stop-process-blocking-a-port)

```powershell
.\.venv\Scripts\python.exe -m robot_pickup --source 0 --display --serial-port COM3 --distance serial
```

For the separate ESP32-CAM, pass its actual MJPEG URL and select the eraser preset. This example uses Espressif CameraWebServer's default stream path and an example board IP. The sensor board still connects by USB. [Espressif CameraWebServer source](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/app_httpd.cpp)

```powershell
.\.venv\Scripts\python.exe -m robot_pickup --source http://192.168.1.50:81/stream --display --config config/eraser.example.json --serial-port COM3 --distance serial
```

In the preview, show the eraser to the camera and move a flat object toward or away from the HC-SR04. Look for the `black-eraser` box and a changing `Beam … cm` reading. `IDLE` is expected while you test telemetry. The beam reading is the sensor's echo, which can come from another surface. Align and calibrate the sensor relative to the camera before using that reading for pickup.

## Test commands on the ESP32

Python decides when to send `TURN`, `MOVE`, `STOP`, or an arm action. The ESP32 receives those commands over USB and returns an acknowledgment with the same sequence number. The sketch's command bench simulates base and arm state so you can test this exchange with just the ESP32.

1. Install **ArduinoJson 7** in Arduino IDE's Library Manager. Open [the sketch](arduino/hcsr04_telemetry/hcsr04_telemetry.ino), set `ENABLE_COMMAND_BENCH` to `1`, and upload with **ESP32 Dev Module** selected. [ArduinoJson installation guide](https://arduinojson.org/v7/how-to/install-arduinojson/)
2. Close the Python preview if open. Open Serial Monitor at **115200 baud**, select **Newline**, and send each line below separately.

```json
{"type":"cmd","seq":1,"command":"STOP","linear":0,"angular":0}
{"type":"cmd","seq":2,"command":"TURN","linear":0,"angular":0.2}
```

Look for `"type":"ack"` with `"accepted":true`, followed by a `"type":"bench"` line showing the simulated command. Once simulated movement starts, the bench watchdog stops it after 500 ms without an accepted command or `HEARTBEAT`. To test the arm message, send the `STOP` line again and then send the second line with `"command":"GRAB"` and `"angular":0`. Look for a `done` message for sequence 2.

The bench only updates simulated actuator state. Real motion requires the actuator wiring and driver code. Connect the receiver to that code so base commands set motor speed, `STOP` disables movement, and each arm action reports `done` after its physical completion. [Arduino command and watchdog contract](docs/arduino-protocol.md)

## Range input and calibration

The serial adapter shares a port for command exchange and range telemetry. `--source 0 --serial-port COM3 --distance serial --display` opens a telemetry-only session with a mock command link. In the preview, `beam` shows the latest valid raw echo. Without a port or network endpoint, a camera session has unknown range.

For network telemetry, copy `config/pickup.example.json` to `config/pickup.local.json` and set `distance.http_url` to your approved sensor endpoint, for example `http://192.168.1.51/distance`. Run from the repository root with `--config config/pickup.local.json --distance http`. Keep custom JSON settings inside `config/` and make sure any links resolve within that directory.

The HTTP adapter connects directly to the configured endpoint and treats redirects as failed readings. The optional `--distance-url` must match `distance.http_url` exactly.

Set `measured_at` to the actual Unix timestamp in seconds at acquisition, using a clock synchronized with the Python host. A repeated timestamp does not refresh the reading. Use `"valid": false` and `"distance_cm": null` for a timeout or no echo.

```json
{"distance_cm": 24.8, "valid": true, "measured_at": 1795424400.25}
```

1. In preview, tune `perception.colors`, `min_area_px`, and `max_area_fraction` with the target under expected lighting. Confirm that target IDs persist over normal motion.
2. Measure the ultrasonic beam with known objects at several positions in the operating range. Set `beam.center_x` and `center_y` to its projected aim point and `half_width` and `half_height` to cover the plausible echoes in the camera image. Image centers use `[-1, 1]` with positive x to the right and positive y down. The overlay crosshair shows the configured aim point. Set `beam.calibrated` to `true` after checking the mount and repeatability.
3. Compare range readings to measured distances. Tune `distance.min_cm`, `max_cm`, `max_age_s`, `median_window`, and `beam.max_skew_s` for the observed range, telemetry rate, and camera latency. An invalid sample clears the median window. The controller also uses the latest raw measurement to stop for a sudden short echo.
4. Measure the range from the sensor face where the stopped gripper can reliably reach the object. Calibrate `control.pickup_cm` and `pickup_window_cm` so the valid interval `[pickup_cm - pickup_window_cm, pickup_cm]` is inside this reach. Tune speed limits, braking margin, settling time, and the number of distinct confirming samples. Test alignment and stopping at low speeds before trying the arm sequence.

Only the selected, visible object can receive a distance estimate. It must cover the calibrated aim point, have a recent coherent sample, and have no other detected bounding box intersect the projected beam. Use a clear, controlled workspace. Invisible clutter, wall and floor echoes, and camera or telemetry buffering can make an association wrong. The tracker can switch IDs when similar objects overlap or cross.

## Arduino integration

Include `--extra serial` when syncing the environment for Arduino integration. Once the Arduino team implements or maps the protocol and has tested its independent watchdog, run a hardware session with a calibrated copy of the configuration.

```sh
python -m robot_pickup --source http://192.168.1.50:81/stream --config config/pickup.local.json --hardware --activate --serial-port COM3 --distance serial --display
```

`--hardware` requires `--activate`, a live camera, a serial port, a real range input, and `beam.calibrated`. The controller starts with an acknowledged `STOP`. Search is stationary with the default `control.search_angular` of `0`. Set `serial.combined_velocity` to `true` after the firmware supports simultaneous linear and angular motion. `RECOVER` attempts a `STOP` and ends the run. Resolve the fault before starting a new attempt.

An acknowledgment confirms acceptance of a command. Each arm step must also report completion. A fresh gripper reading correlated with the last arm step determines the pickup result. Exit code `0` means verified pickup or normal preview exit, `2` means an error or failed pickup, `3` means an unverified pickup, and `130` means the operator ended an active run.

The Arduino team needs to agree on transport and baud rate, velocity units and signs or PWM mapping, arm completion feedback and timings, the range telemetry format and sample rate, gripper feedback, and watchdog behavior. See the [Arduino protocol proposal](docs/arduino-protocol.md) for the contract used by the Python adapter.
