"""Mocked observation adapter tests; no ROS/device processes are launched."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch


@unittest.skipUnless(sys.platform=='linux','Observation adapter runs inside WSL')
class ObservationAdapterTests(unittest.TestCase):
    def check(self,duplicate=False,idle=False,serial_fail=False,open_fail=False,pose_fail=False):
        import control_panel_action as adapter
        root=Path(__file__).resolve().parent
        c=json.loads((root/'config.json').read_text())
        node=Mock()
        node.count_publishers.side_effect=lambda name: 1 if name=='/RobotMove' or duplicate else 0
        node.publisher.get_subscription_count.return_value=0
        node.spin.side_effect=lambda:time.sleep(.02)
        node.read_start.return_value=Mock(pose=tuple(c['observation_pose']))
        order=[]
        def move(*args,**kwargs):
            order.append('move')
            if idle: raise RuntimeError('控制器状态不允许运动：Idle')
        node.move_to.side_effect=move
        if pose_fail:node.read_start.return_value.pose=tuple([c['observation_pose'][0]+.01]+c['observation_pose'][1:])
        devices=Mock()
        def connect():
            order.append('connect')
            if serial_fail:raise RuntimeError('夹爪串口不可用')
        def open_gripper(close):
            self.assertFalse(close)
            order.append('open')
            if open_fail:raise TimeoutError('夹爪真实动作完成反馈超时')
        devices.connect_gripper.side_effect=connect
        devices.action.side_effect=open_gripper
        ros=Mock();ros.ok.return_value=True
        process=Mock(pid=123456789);process.poll.return_value=None
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'out'
            args=['control_panel_action.py','--config',str(root/'config.json'),'--output',str(output)]
            with patch.object(sys,'argv',args),patch.dict(sys.modules,{'rclpy':ros}), \
                 patch.object(adapter,'make_node',return_value=node), \
                 patch.object(adapter,'ObservationGripper',return_value=devices), \
                 patch.object(adapter.fcntl,'flock'), \
                 patch.object(adapter.subprocess,'Popen',return_value=process) as start, \
                 patch.object(adapter.os,'killpg'):
                code=adapter.main()
            result=json.loads((output/'result.json').read_text())
        node.destroy_node.assert_called_once()
        if duplicate:
            self.assertEqual(code,2);start.assert_not_called();node.move_to.assert_not_called()
            self.assertIn('已有运动程序',result['reason'])
            devices.action.assert_not_called()
        else:
            self.assertEqual(start.call_args.args[0][:4],['ros2','run','estun_segmented','bridge'])
            devices.gripper.disconnect.assert_called_once()
            if serial_fail:
                self.assertEqual(code,2);node.move_to.assert_not_called();devices.action.assert_not_called()
                self.assertFalse(result['observation_reached'])
                return
            node.move_to.assert_called_once_with(c['observation_pose'],stage='all',execute=True)
            if idle or pose_fail:
                self.assertEqual(code,2);devices.action.assert_not_called()
                self.assertFalse(result['observation_reached'])
            elif open_fail:
                self.assertEqual(code,2);self.assertTrue(result['observation_reached'])
                self.assertFalse(result['gripper_open_completed'])
                devices.action.assert_called_once_with(False)
            else:
                self.assertEqual(code,0);self.assertTrue(result['success'])
                self.assertTrue(result['observation_reached']);self.assertTrue(result['gripper_open_completed'])
                self.assertEqual(order,['connect','move','open'])
                self.assertLessEqual(result['position_error_m'],c['motion']['position_tolerance'])
            if idle:
                self.assertEqual(code,2);self.assertIn('Idle',result['reason'])
    def test_original_motion_adapter_is_used(self): self.check()
    def test_duplicate_session_refused_before_motion(self): self.check(duplicate=True)
    def test_idle_failure_is_reported_without_resend(self): self.check(idle=True)
    def test_serial_fault_prevents_arm_motion(self):self.check(serial_fail=True)
    def test_pose_error_prevents_opening(self):self.check(pose_fail=True)
    def test_open_timeout_is_failure_after_return(self):self.check(open_fail=True)


if __name__=='__main__': unittest.main(verbosity=2)
