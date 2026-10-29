"""联动顺序、到位门禁和一次发送语义；不连接设备。"""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_pick import Gripper, MotionError, Settings, load_target, main, pick, target_pose

TARGET = (0.4, 0.1, 0.3, 3.14, 0, 0)


class PickTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.motion = Mock()
        self.motion.move_to.side_effect = lambda *a, **k: self.events.append('move_completed')
        self.motion.read_start.side_effect = lambda: (
            self.events.append('arrival_confirmed') or SimpleNamespace(pose=TARGET))
        self.gripper = Mock()
        self.gripper.check_ready.side_effect = lambda: self.events.append('query')
        self.gripper.close_once.side_effect = lambda: self.events.append('close') or 'written'
        self.journal = Mock()

    def run_pick(self, execute=True):
        pick(self.motion, self.gripper, TARGET, Settings(), self.journal, execute)

    def test_close_only_after_motion_and_final_confirmation(self):
        self.run_pick()
        self.assertEqual(self.events, ['query', 'move_completed', 'arrival_confirmed', 'close'])
        self.gripper.close_once.assert_called_once_with()
        self.motion.move_to.assert_called_once_with(TARGET, stage='all', execute=True)

    def test_preview_never_calls_gripper(self):
        self.run_pick(False)
        self.gripper.check_ready.assert_not_called()
        self.gripper.close_once.assert_not_called()
        self.motion.read_start.assert_not_called()
        self.motion.move_to.assert_called_once_with(TARGET, stage='all', execute=False)

    def test_missing_gripper_feedback_prevents_arm_motion(self):
        self.gripper.check_ready.side_effect = MotionError('no feedback / dry run')
        with self.assertRaises(MotionError):
            self.run_pick()
        self.motion.move_to.assert_not_called()
        self.gripper.close_once.assert_not_called()

    def test_motion_failure_never_closes(self):
        for reason in ('fault', 'feedback timeout', 'arrival timeout'):
            with self.subTest(reason=reason):
                self.motion.move_to.side_effect = MotionError(reason)
                with self.assertRaises(MotionError):
                    self.run_pick()
                self.gripper.close_once.assert_not_called()

    def test_lost_final_feedback_never_closes(self):
        self.motion.read_start.side_effect = MotionError('stale feedback')
        with self.assertRaises(MotionError):
            self.run_pick()
        self.gripper.close_once.assert_not_called()

    def test_wrong_final_position_or_orientation_never_closes(self):
        for axis in range(6):
            with self.subTest(axis=axis):
                wrong = list(TARGET)
                wrong[axis] += 0.1
                self.motion.read_start.side_effect = None
                self.motion.read_start.return_value = SimpleNamespace(pose=wrong)
                with self.assertRaises(MotionError):
                    self.run_pick()
                self.gripper.close_once.assert_not_called()

    def test_close_failure_is_not_retried(self):
        self.gripper.close_once.side_effect = MotionError('serial write failed')
        with self.assertRaises(MotionError):
            self.run_pick()
        self.gripper.close_once.assert_called_once_with()

    def test_relative_target_computed_once_from_current_feedback(self):
        start = (0.3, 0, 0.35, 3.14, 0.7853981633974483, 0)
        self.motion.read_start.side_effect = [SimpleNamespace(pose=start), SimpleNamespace(pose=TARGET)]
        delta = {'delta': [0.1, 0.1, -0.05, 0, -0.7853981633974483, 0]}
        pick(self.motion, self.gripper, delta, Settings(), self.journal, execute=True)
        args = self.motion.move_to.call_args.args[0]
        for actual, expected in zip(args, TARGET):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.motion.read_start.call_count, 2)
        self.gripper.close_once.assert_called_once_with()


class ConfigTests(unittest.TestCase):
    def load(self, data, name='part'):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'targets.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            return load_target(path, name)

    def test_named_pose_and_settings(self):
        target, settings = self.load({'targets': {'part': TARGET}, 'motion': {'linear_speed': 0.002}})
        self.assertEqual(tuple(target), TARGET)
        self.assertEqual(settings.linear_speed, 0.002)

    def test_unknown_name(self):
        with self.assertRaises(ValueError):
            self.load({'targets': {}})

    def test_invalid_poses(self):
        for target in (None, 'xyz', [1] * 5, [True] * 6, [float('nan')] * 6,
                       {'delta': [1] * 5}, {'delta': [0] * 6, 'bad': 1}):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.load({'targets': {'part': target}})

    def test_requested_relative_config(self):
        target, _ = load_target(Path(__file__).resolve().parents[1] / 'pick_targets.json', '抓取点')
        result = target_pose(target, TARGET)
        expected = (0.5, 0.2, 0.25, 3.14, -0.7853981633974483, 0)
        for actual, wanted in zip(result, expected):
            self.assertAlmostEqual(actual, wanted)

    def test_invalid_settings(self):
        for motion in ({'linear_speed': -1}, {'unknown': 1}, []):
            with self.subTest(motion=motion), self.assertRaises((TypeError, ValueError)):
                self.load({'targets': {'part': TARGET}, 'motion': motion})

    def test_offline_cli_needs_no_ros_or_gripper(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'targets.json'
            path.write_text(json.dumps({'targets': {'part': TARGET}}), encoding='utf-8')
            with patch('run_pick.make_node') as make_node, patch('run_pick.Journal'):
                self.assertEqual(main(['part', '--config', str(path), '--start', *map(str, TARGET)]), 0)
                make_node.assert_not_called()


class ServiceTests(unittest.TestCase):
    def adapter(self):
        adapter = Gripper.__new__(Gripper)
        adapter.motion = Mock()
        adapter.request_type = lambda: None
        adapter.query = Mock()
        adapter.close = Mock()
        return adapter

    def test_timeout_sends_once(self):
        adapter = self.adapter()
        adapter.close.call_async.return_value.done.return_value = False
        with patch('run_pick.time.monotonic', side_effect=[0, 1, 4]):
            with self.assertRaisesRegex(MotionError, '不重发'):
                adapter.close_once()
        adapter.close.call_async.assert_called_once()

    def test_failed_response_sends_once(self):
        adapter = self.adapter()
        adapter.close.call_async.return_value.result.return_value = SimpleNamespace(
            success=False, message='failed')
        with self.assertRaisesRegex(MotionError, 'failed'):
            adapter.close_once()
        adapter.close.call_async.assert_called_once()

    def test_service_disappears_no_send(self):
        adapter = self.adapter()
        adapter.close.service_is_ready.return_value = False
        with self.assertRaises(MotionError):
            adapter.close_once()
        adapter.close.call_async.assert_not_called()


if __name__ == '__main__':
    unittest.main()
