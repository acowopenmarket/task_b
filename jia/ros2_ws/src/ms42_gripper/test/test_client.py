import unittest

from ms42_gripper.client import CommunicationError, FeedbackTimeout, GripperClient
from ms42_gripper.protocol import bcc, query_frame


class FakeSerial:
    def __init__(self, **kwargs):
        self.settings = kwargs
        self.is_open = True
        self.writes = []
        self.pending = bytearray()
        self.reset_count = 0
        self.short_write = False
        self.fail_read = False
        payload = bytes.fromhex('01 01 00 64 00 01 11 70')  # 70000 raw
        self.reply = payload + bytes([bcc(payload)])

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, data):
        self.writes.append(data)
        if data[2] == 0:
            self.pending.extend(self.reply)
        return len(data) - int(self.short_write)

    def read(self, count):
        if self.fail_read:
            raise OSError('Cable disconnected')
        # Intentionally fragment replies into 2-byte chunks.
        count = min(2, count)
        result = bytes(self.pending[:count])
        del self.pending[:count]
        return result

    def reset_input_buffer(self):
        self.reset_count += 1
        self.pending.clear()

    def close(self):
        self.is_open = False


class ClientTests(unittest.TestCase):
    def hardware_client(self):
        client = GripperClient(dry_run=False, serial_factory=FakeSerial,
                               feedback_timeout=0.01)
        client.connect()
        return client

    def test_dry_run_never_opens_serial(self):
        def forbidden(**kwargs):
            self.fail('Dry run opened a serial port')
        client = GripperClient(serial_factory=forbidden)
        client.connect()
        self.assertFalse(client.connected)
        self.assertEqual(len(client.move(True, 90, 10)), 11)
        with self.assertRaises(FeedbackTimeout):
            client.query()

    def test_connect_sends_no_motion(self):
        client = self.hardware_client()
        self.assertTrue(client.connected)
        self.assertEqual(client.serial_port.writes, [])
        self.assertEqual(client.serial_port.settings['baudrate'], 115200)
        self.assertEqual(client.serial_port.settings['timeout'], 0)

    def test_moves_are_one_shot(self):
        client = self.hardware_client()
        frame = client.move(True, 90, 10)
        self.assertEqual(client.serial_port.writes, [frame])
        self.assertEqual(frame[2], 2)

    def test_configured_limits_reject_before_write(self):
        client = self.hardware_client()
        for angle, speed in [(1872.1, 10), (90, 20.1), (float('nan'), 10)]:
            with self.subTest(angle=angle, speed=speed), self.assertRaises(ValueError):
                client.move(True, angle, speed)
        self.assertEqual(client.serial_port.writes, [])

    def test_disconnected_move_not_queued(self):
        client = self.hardware_client()
        client.disconnect()
        with self.assertRaises(CommunicationError):
            client.move(False, 90, 10)
        client.connect()
        self.assertEqual(client.serial_port.writes, [])

    def test_query_fragmented_feedback_and_stale_input(self):
        client = self.hardware_client()
        client.serial_port.pending.extend(b'old input')
        state = client.query()
        self.assertEqual(state.raw_angle, 70000)
        self.assertEqual(state.angle_deg, 7000.0)
        self.assertEqual(client.serial_port.writes, [query_frame(1)])
        self.assertEqual(client.serial_port.reset_count, 1)

    def test_timeout_does_not_resend_motion_or_disconnect(self):
        client = self.hardware_client()
        client.serial_port.reply = b''
        with self.assertRaises(FeedbackTimeout):
            client.query()
        self.assertTrue(client.connected)
        self.assertEqual(client.serial_port.writes, [query_frame(1)])

    def test_short_write_disconnects_without_replay(self):
        client = self.hardware_client()
        port = client.serial_port
        port.short_write = True
        with self.assertRaises(CommunicationError):
            client.move(True, 90, 10)
        self.assertFalse(client.connected)
        self.assertFalse(port.is_open)
        self.assertEqual(len(port.writes), 1)

    def test_read_error_disconnects(self):
        client = self.hardware_client()
        client.serial_port.fail_read = True
        with self.assertRaises(OSError):
            client.query()
        self.assertFalse(client.connected)

    def test_shutdown_does_not_send_undocumented_stop(self):
        client = self.hardware_client()
        port = client.serial_port
        client.disconnect()
        self.assertEqual(port.writes, [])
        self.assertFalse(port.is_open)

    def test_bad_configuration(self):
        for config in [
            {'max_angle_deg': 7000}, {'max_speed_rad_s': 43},
            {'motor_id': 0}, {'subdivision': 3}, {'feedback_timeout': 0},
            {'port': ''}, {'baudrate': 0},
        ]:
            with self.subTest(config=config), self.assertRaises(ValueError):
                GripperClient(**config)


if __name__ == '__main__':
    unittest.main()
