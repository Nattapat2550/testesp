import json
import unittest

from robot_pickup.communication.commands import (
    STOP, Ack, ActionDone, Command, CommandKind, CommandLink, LinkSettings, MockArduino, Transmission,
)
from robot_pickup.communication.serial_adapter import SerialArduino


class MemoryPort:
    def __init__(self):
        self.received = b""
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.received)

    def read(self, size):
        data, self.received = self.received[:size], self.received[size:]
        return data

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


class CommunicationTests(unittest.TestCase):
    def test_deduplicates_motion_and_sends_heartbeat(self):
        arduino = MockArduino()
        link = CommandLink(arduino, LinkSettings())
        command = Command(CommandKind.VELOCITY, 0.3, -0.1)
        link.set_motion(command, 0.0)
        link.poll(0.02)
        for now in (0.04, 0.08, 0.13):
            link.set_motion(command, now)
            link.keepalive(now)
        self.assertEqual([m.command.kind for m in arduino.history], [CommandKind.VELOCITY, CommandKind.HEARTBEAT])
        self.assertEqual(arduino.motion, command)

    def test_stop_preempts_unacknowledged_motion_and_arm_requires_stopped_ack(self):
        arduino = MockArduino(action_time_s=0.1)
        link = CommandLink(arduino, LinkSettings())
        link.set_motion(Command(CommandKind.MOVE, 0.2), 0.0)
        self.assertIsNone(link.start_action(CommandKind.GRAB, 0.01))
        link.set_motion(STOP, 0.02)
        self.assertEqual(arduino.motion, STOP)
        self.assertIsNone(link.start_action(CommandKind.GRAB, 0.03))
        link.poll(0.04)
        sequence = link.start_action(CommandKind.GRAB, 0.05)
        self.assertIsNotNone(sequence)
        link.poll(0.06)
        self.assertFalse(link.action_done(sequence))
        self.assertIsNone(link.start_action(CommandKind.LIFT, 0.07))
        link.poll(0.16)
        self.assertTrue(link.action_done(sequence))

    def test_unacknowledged_stop_suppresses_heartbeat(self):
        arduino = MockArduino()
        link = CommandLink(arduino, LinkSettings())
        link.set_motion(Command(CommandKind.MOVE, 0.2), 0.0)
        link.poll(0.01)
        link.set_motion(STOP, 0.03)
        link.keepalive(0.17)
        self.assertEqual([sent.command.kind for sent in arduino.history], [CommandKind.MOVE, CommandKind.STOP])

    def test_serial_writes_commands_and_decodes_split_lines(self):
        port = MemoryPort()
        arduino = SerialArduino(port, supports_combined_velocity=True)
        arduino.send(Transmission(4, Command(CommandKind.VELOCITY, 0.25, -0.1), 0.0))
        self.assertTrue(port.writes[0].endswith(b"\n"))
        self.assertEqual(json.loads(port.writes[0]), {
            "type": "cmd", "seq": 4, "command": "VELOCITY", "linear": 0.25, "angular": -0.1,
        })
        port.received = b'Arduino ready\n{"type":"ack","seq":4,'
        self.assertEqual(arduino.poll(0.0), [])
        port.received = b'"accepted":true}\n{"type":"done","seq":5,"success":true}\n'
        self.assertEqual(arduino.poll(0.03), [Ack(4, True), ActionDone(5, True)])
        arduino.close()
        self.assertTrue(port.closed)

    def test_serial_range_sample_ids_keep_duplicates_from_refreshing_data(self):
        port = MemoryPort()
        arduino = SerialArduino(port)
        telemetry = b'{"type":"distance","distance_cm":26,"sample_id":20,"valid":true,"age_ms":50}\n'
        port.received = telemetry
        arduino.poll(1.0)
        sample = arduino.latest(1.0)
        self.assertTrue(sample.valid)
        self.assertAlmostEqual(sample.measured_at, 0.95)
        port.received = telemetry
        arduino.poll(1.5)
        self.assertEqual(arduino.latest(1.5), sample)
        port.received = telemetry.replace(b'"sample_id":20', b'"sample_id":19')
        arduino.poll(1.6)
        self.assertFalse(arduino.latest(1.6).valid)

    def test_serial_backlog_and_partial_writes_are_link_errors(self):
        port = MemoryPort()
        arduino = SerialArduino(port)
        port.received = b"x" * 4097
        with self.assertRaises(OSError):
            arduino.poll(1.0)
        port.write = lambda data: len(data) - 1
        with self.assertRaises(OSError):
            arduino.send(Transmission(1, STOP, 1.0))
