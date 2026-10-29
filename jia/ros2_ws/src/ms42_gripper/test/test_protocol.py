import unittest

from ms42_gripper.protocol import FeedbackParser, bcc, move_frame, query_frame


def feedback(angle=3600, speed=100, reached=0, motor_id=1):
    payload = (bytes([motor_id, reached]) + speed.to_bytes(2, 'big')
               + angle.to_bytes(4, 'big'))
    return payload + bytes([bcc(payload)])


class ProtocolTests(unittest.TestCase):
    def test_manual_close(self):
        self.assertEqual(move_frame(1, True, 1872.0, 20.0),
                         bytes.fromhex('7b 01 02 01 20 49 20 00 c8 f8 7d'))

    def test_manual_open(self):
        self.assertEqual(move_frame(1, False, 1872.0, 20.0),
                         bytes.fromhex('7b 01 02 00 20 49 20 00 c8 f9 7d'))

    def test_quick_start(self):
        self.assertEqual(move_frame(1, True, 1800.0, 30.0),
                         bytes.fromhex('7b 01 02 01 20 46 50 01 2c 62 7d'))

    def test_manual_query(self):
        self.assertEqual(query_frame(1), bytes.fromhex('7b 01 00 00 00 00 00 00 00 7a 7d'))

    def test_units_and_rounding(self):
        frame = move_frame(2, True, 90.05, 10.05, 16)
        self.assertEqual(int.from_bytes(frame[5:7], 'big'), 901)
        self.assertEqual(int.from_bytes(frame[7:9], 'big'), 101)
        self.assertEqual(bcc(frame[:9]), frame[9])

    def test_wire_limits(self):
        frame = move_frame(255, False, 6553.5, 42.0)
        self.assertEqual(frame[5:7], b'\xff\xff')

    def test_invalid_ids(self):
        for motor_id in [0, 256, -1, True, 1.0]:
            with self.subTest(motor_id=motor_id), self.assertRaises(ValueError):
                query_frame(motor_id)

    def test_invalid_motion_fields(self):
        cases = [
            (True, 0, 10, 32), (True, -1, 10, 32), (True, 6553.6, 10, 32),
            (True, float('nan'), 10, 32), (True, float('inf'), 10, 32),
            (True, 90, 0, 32), (True, 90, -1, 32), (True, 90, 42.1, 32),
            (True, 90, float('nan'), 32), (True, 90, 10, 3),
            (True, 90, 10, 32.0), (1, 90, 10, 32), (True, True, 10, 32),
        ]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                move_frame(1, *case)

    def test_manual_feedback(self):
        parser = FeedbackParser(1)
        state = parser.feed(bytes.fromhex('01 00 00 64 00 00 0e 10 7b'))[0]
        self.assertEqual(state.speed_rad_s, 10.0)
        self.assertEqual(state.angle_deg, 360.0)
        self.assertFalse(state.target_reached)

    def test_manual_feedback_table_arithmetic(self):
        state = FeedbackParser(1).feed(bytes.fromhex('01 01 00 00 00 00 8c a0 2c'))[0]
        # Manual prose says 360; bytes actually encode 3600 degrees.
        self.assertEqual(state.angle_deg, 3600.0)
        self.assertTrue(state.target_reached)

    def test_every_split_boundary(self):
        frame = feedback(angle=0x12345678, reached=1)
        for split in range(1, 9):
            parser = FeedbackParser(1)
            self.assertEqual(parser.feed(frame[:split]), [])
            states = parser.feed(frame[split:])
            self.assertEqual(states[0].raw_angle, 0x12345678)

    def test_multiple_frames_and_noise(self):
        parser = FeedbackParser(1)
        states = parser.feed(b'\xff\xff\xff' + feedback(1000) + feedback(70000))
        self.assertEqual([state.raw_angle for state in states], [1000, 70000])
        self.assertLess(len(parser.buffer), 9)

    def test_checksum_corruption(self):
        corrupt = bytearray(feedback())
        corrupt[-1] ^= 0x80
        parser = FeedbackParser(1)
        self.assertEqual(parser.feed(corrupt), [])
        self.assertEqual(parser.feed(feedback(12345))[0].raw_angle, 12345)

    def test_wrong_id_and_invalid_flag(self):
        parser = FeedbackParser(1)
        self.assertEqual(parser.feed(feedback(motor_id=2)), [])
        parser.clear()
        self.assertEqual(parser.feed(feedback(reached=2)), [])

    def test_full_unsigned_32_bit_range(self):
        state = FeedbackParser(1).feed(feedback(angle=0xFFFFFFFF, speed=65535))[0]
        self.assertEqual(state.raw_angle, 4294967295)
        self.assertEqual(state.raw_speed, 65535)

    def test_noise_does_not_accumulate(self):
        parser = FeedbackParser(1)
        self.assertEqual(parser.feed(b'\xff' * 10000), [])
        self.assertLess(len(parser.buffer), 9)


if __name__ == '__main__':
    unittest.main()
