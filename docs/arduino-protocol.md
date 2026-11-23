# Arduino serial protocol proposal

The Python adapter implements this proposal for integration with the Arduino team's firmware. Agree on the final wire format and actuator semantics before enabling hardware control. Firmware work belongs on the Arduino side.

The [HC-SR04 bench sketch](../arduino/hcsr04_telemetry/hcsr04_telemetry.ino) sends the distance messages below over USB at 115200 baud. It uses `pulseIn` with a 30 ms timeout. For the motor firmware, schedule range measurement so echo timing and serial output leave time for the command parser and watchdog.

## Framing and timing

Use UTF-8 JSON objects terminated by `\n` over a serial port, initially 115200 baud, 8 data bits, no parity, and one stop bit. Keep each line at or below 1024 bytes. Send telemetry promptly so the host can use its receipt time and the sample age. The Python adapter faults on a receive backlog above 4096 bytes. It ignores housekeeping text and unknown message types. New host sessions start at `seq: 1` with `STOP`. Clear stale input on session startup and keep actuators stopped at startup and after reset.

## Python to Arduino

```json
{"type":"cmd","seq":1,"command":"STOP","linear":0.0,"angular":0.0}
{"type":"cmd","seq":2,"command":"VELOCITY","linear":0.25,"angular":-0.10}
{"type":"cmd","seq":3,"command":"HEARTBEAT","linear":0.0,"angular":0.0}
```

| Command | Behavior |
| --- | --- |
| `MOVE` | Set linear velocity and clear rotation. |
| `TURN` | Rotate in place and clear forward motion. |
| `VELOCITY` | Apply both axes simultaneously if supported. |
| `STOP` | Stop the base and safely halt any arm action in progress. Give `STOP` priority. ACK only once the base has stopped and the arm is safe. |
| `LOWER`, `GRAB`, `LIFT`, `RELEASE` | Start one arm or gripper action while the base is stopped. Send acceptance and then a completion message. |
| `HEARTBEAT` | Renew the command watchdog without altering motion or reissuing an arm action. |

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
