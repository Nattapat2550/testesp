# Arduino serial protocol proposal

The Python adapter implements this proposal for integration with the Arduino team's firmware. Agree on the final wire format and actuator semantics before enabling hardware control. Firmware work belongs on the Arduino side.

The [HC-SR04 bench sketch](../arduino/hcsr04_telemetry/hcsr04_telemetry.ino) sends the distance messages below over USB at 115200 baud. It supports a classic ESP32 DevKit through Arduino IDE's ESP32 Dev Module target and Uno or classic Nano through the AVR targets. It uses `pulseIn` with a 30 ms timeout. For the motor firmware, schedule range measurement so echo timing and serial output leave time for the command parser and watchdog.

Set `ENABLE_COMMAND_BENCH` to `1` on the ESP32 to enable the serial command test. The bench acknowledges accepted commands, prints simulated commands as `type: bench`, and reports simulated arm completion. It allows a manual pause after the initial `STOP` and runs a 500 ms watchdog during simulated motion or an arm action. Real actuator firmware should implement the watchdog and completion semantics below using the actual motor and arm states.

## Framing and timing

Use UTF-8 JSON objects terminated by `\n` over a serial port, initially 115200 baud, 8 data bits, no parity, and one stop bit. Keep each line at or below 1024 bytes. Send telemetry promptly so the host can use its receipt time and the sample age. The Python adapter faults on a receive backlog above 4096 bytes. It ignores housekeeping text and unknown message types. New host sessions start at `seq: 1` with `STOP`. Clear stale input on session startup and keep actuators stopped at startup and after reset.

## Python to Arduino

```json
{"type":"cmd","seq":1,"command":"STOP","linear":0.0,"angular":0.0}
{"type":"cmd","seq":2,"command":"VELOCITY","linear":0.25,"angular":-0.10}
{"type":"cmd","seq":3,"command":"HEARTBEAT","linear":0.0,"angular":0.0}
```

| Command                            | Behavior                                                                                                                                |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `MOVE`                             | Set linear velocity and clear rotation.                                                                                                 |
| `TURN`                             | Rotate in place and clear forward motion.                                                                                               |
| `VELOCITY`                         | Apply both axes simultaneously if supported.                                                                                            |
| `STOP`                             | Stop the base and safely halt any arm action in progress. Give `STOP` priority. ACK only once the base has stopped and the arm is safe. |
| `LOWER`, `GRAB`, `LIFT`, `RELEASE` | Start one arm or gripper action while the base is stopped. Send acceptance and then a completion message.                               |
| `HEARTBEAT`                        | Renew the command watchdog without altering motion or reissuing an arm action.                                                          |

Speeds are normalized to `[-1, 1]`. Positive linear is forward and positive angular turns toward the right side of the camera image. Clamp the physical speeds in firmware and agree on the mapping to wheel velocities or PWM. Ignore the two speed fields for commands that do not use them. The host sends changes to base motion and sends heartbeats during quiet periods once outstanding ACKs arrive. Each arm step gets one command.

## Arduino to Python

```json
{"type":"ack","seq":1,"accepted":true}
{"type":"done","seq":8,"success":true}
{"type":"distance","sample_id":13,"distance_cm":23.6,"valid":true,"age_ms":5}
{"type":"distance","sample_id":14,"distance_cm":null,"valid":false,"age_ms":1}
{"type":"grip","seq":8,"holding":true,"age_ms":3}
```

- Acknowledge every command, including `STOP` and `HEARTBEAT`, using its `seq`. Send `accepted: false` if the command is unsupported, invalid, or unsafe to start. Python stops on a rejection or missing ACK. The default ACK deadline is 350 ms.
- Send `done` with the arm command's `seq` after the action actually finishes. Use `success: false` if an actuator fails. An accepted arm command alone cannot advance the pickup sequence.
- Send range telemetry at a steady rate, such as 10–20 Hz with the example settings. Use a strictly increasing `sample_id` per new acquisition for the session. Report timeouts and invalid echoes explicitly. `age_ms` is the elapsed time since acquisition when the message is sent. Avoid buffering and restart the host session after a board reset.
- If there is an actual grip or object-presence sensor, sample it after the final arm action and report `grip` with that action's `seq`. `holding: false` means the sensor observed an empty gripper. Omit `grip` when feedback is unavailable, which yields an unverified result after the timeout.

## Firmware watchdog

Python cannot guarantee delivery of a stop once the link drops, the host freezes, or a write fails. Implement a watchdog in the Arduino motor and arm loop independent of serial parsing. If the deadline passes without an accepted command or heartbeat, stop the base, safely halt any arm motion, and handle the gripper according to the robot's holding requirements. Keep motion disabled until the host establishes a new stopped session. Invalid messages must leave the deadline unchanged.

Choose and test a watchdog interval against stopping distance, controller rate, and serial jitter. The default heartbeat is 120 ms. Verify the watchdog by disconnecting the serial cable during a controlled low-speed test and during an arm action.

## Target position output

With `--position-only`, Python publishes a `target` message to the terminal and optionally to `--serial-port`. The ESP32 firmware can use these messages as camera input for its own controller. Use 115200 baud by default and read one JSON object per newline.

```json
{"type":"target","found":true,"label":"black-eraser","object_id":1,"confidence":0.73,"cx":0.25,"cy":-0.5,"x_px":400.0,"y_px":120.0,"frame_width":640,"frame_height":480}
{"type":"target","found":false}
```

- `found: true` reports the center of the selected tracked object. `cx` and `cy` are normalized to `[-1, 1]`, with `(0, 0)` at the center. Positive x points right and positive y points down. Pixel coordinates use the top-left of the frame as the origin.
- `object_id` identifies the track within the current run. `confidence` is the detector's confidence in `[0, 1]` when available, or `null` for HSV. The frame dimensions are in pixels.
- Python publishes up to 10 fresh position messages per second. It sends `found: false` when the target disappears, the camera data gets stale, or the stream ends. On an interruption, the ESP32 should also expire the last position locally when no fresh message arrives within its chosen timeout. Reset the stored position immediately on `found: false`.
- The message supplies an image position. The ESP32 firmware owns its motor and gripper decisions and can read the ultrasonic sensor directly for approach distance.

Press **P** in the position preview with the eraser at the gripper's reachable spot. The saved `pickup-point.json` contains `pickup_cx` and `pickup_cy` for the current camera mount. Copy those values into the ESP32 firmware. The image-space correction is `cx - pickup_cx` and `cy - pickup_cy`. Measure the allowed error and check the wheel directions on the robot before running the grasp. The saved point is local to the PC and the target stream keeps its existing fields.

### ESP32 receiver example

The [target receiver sketch](../arduino/esp32_target_receiver/esp32_target_receiver.ino) reads the USB messages and prints its decisions. Install ArduinoJson 7, select **ESP32 Dev Module** in Arduino IDE, and upload the sketch for a bench test. Open Serial Monitor with **115200 baud** and **Newline** selected. With the example pickup point `(0.10, 0.40)`, send these lines one at a time. [ArduinoJson installation guide](https://arduinojson.org/v7/how-to/install-arduinojson/)

| Serial input                                         | Expected decision |
| ---------------------------------------------------- | ----------------- |
| `{"type":"target","found":true,"cx":0.30,"cy":0.40}` | `TURN_RIGHT`      |
| `{"type":"target","found":true,"cx":0.11,"cy":0.20}` | `APPROACH`        |
| `{"type":"target","found":true,"cx":0.11,"cy":0.39}` | `READY_TO_PICK`   |
| `{"type":"target","found":false}`                    | `STOP`            |

The sketch prints a decision when it changes. The target expires after 500 ms without a fresh message. Close Serial Monitor before running Python with `--position-only --serial-port` so Python can open that port.

### Example output

In the Python terminal, a run using `--position-only --serial-port COM3` can look like this as the eraser comes into view and then disappears.

```text
INFO Target position output using YOLO (best.pt)
INFO Sending target positions on COM3 at 115200 baud
{"type":"target","found":false}
{"type":"target","found":true,"label":"black-eraser","object_id":1,"confidence":0.75,"cx":0.3,"cy":0.4,"x_px":416.0,"y_px":336.0,"frame_width":640,"frame_height":480}
{"type":"target","found":false}
```

Python writes the JSON lines to USB. To see the ESP32's reply in Serial Monitor, close Python, open the monitor at **115200 baud** with **Newline** selected, and send just the `found:true` line from the example above. With the sketch's example pickup point `(0.10, 0.40)`, you can see this sequence.

```text
STOP
TURN_RIGHT
STOP
```

The first `STOP` is the sketch waiting for a target. `TURN_RIGHT` comes from `cx` being 0.20 to the right of the pickup point. The final `STOP` appears after 500 ms without a fresh target message.

For the motor firmware, replace the example pickup coordinates with the values you taught. The example assumes floor targets move lower in the image as the car approaches. Verify this on the mounted camera and check clearance before driving. Connect the decisions to the actual motor driver and arm code. On `READY_TO_PICK`, stop the base and start the grasp once, then wait for the grasp to finish before accepting another pickup.
