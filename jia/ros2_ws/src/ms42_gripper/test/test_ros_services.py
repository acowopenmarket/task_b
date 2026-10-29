"""Real ROS dry-run integration checks; skipped when ROS is unavailable."""

import unittest
from unittest.mock import patch

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from ms42_gripper_interfaces.srv import Move
    from ms42_gripper.node import GripperNode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, 'Requires ROS 2 and built ms42_gripper_interfaces')
class RosServiceTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.driver = GripperNode()  # Defaults to dry-run; never opens hardware.
        self.probe = Node('ms42_test_client')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.driver)
        self.executor.add_node(self.probe)

    def tearDown(self):
        self.executor.remove_node(self.driver)
        self.executor.remove_node(self.probe)
        self.executor.shutdown()
        self.driver.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()

    def call(self, service_type, name, request):
        client = self.probe.create_client(service_type, '/ms42_gripper/' + name)
        self.assertTrue(client.wait_for_service(timeout_sec=2.0))
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=2.0)
        self.assertTrue(future.done(), 'Service timed out')
        return future.result()

    def test_open_close_are_dry_run(self):
        expected = {
            'open': '7b 01 02 00 20 49 20 00 c8 f9 7d',
            'close': '7b 01 02 01 20 49 20 00 c8 f8 7d',
        }
        with patch.object(self.driver.client, '_write') as write:
            for name, frame in expected.items():
                response = self.call(Trigger, name, Trigger.Request())
                self.assertTrue(response.success)
                self.assertIn('Dry-run', response.message)
                self.assertEqual(write.call_args.args[0], bytes.fromhex(frame))
            self.assertEqual(write.call_count, 2)
        self.assertFalse(self.driver.client.connected)

    def test_move_accepts_valid_and_rejects_invalid(self):
        request = Move.Request(close=True, angle_deg=90.0, speed_rad_s=10.0)
        self.assertTrue(self.call(Move, 'move', request).success)
        request.angle_deg = float('nan')
        self.assertFalse(self.call(Move, 'move', request).success)
        request.angle_deg = 2000.0
        self.assertFalse(self.call(Move, 'move', request).success)

    def test_query_does_not_invent_feedback(self):
        response = self.call(Trigger, 'query', Trigger.Request())
        self.assertFalse(response.success)
        self.assertIn('Dry-run', response.message)
        self.assertIsNone(self.driver.last_feedback)


if __name__ == '__main__':
    unittest.main()
