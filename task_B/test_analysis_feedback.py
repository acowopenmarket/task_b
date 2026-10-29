"""Offline scheduling regression: no ROS, serial, or hardware connections."""
import threading
import time
import unittest
from unittest.mock import patch

from hardware import Devices


class AnalysisFeedbackTests(unittest.TestCase):
    def device(self):
        return Devices({},None,None)

    def test_slow_analysis_keeps_feedback_checks_on_caller(self):
        device=self.device()
        caller=threading.get_ident()
        checks=[]
        def healthy(camera=True):
            self.assertFalse(camera)
            self.assertEqual(threading.get_ident(),caller)
            checks.append(time.monotonic())
            time.sleep(.01)
        def slow(*args,**kwargs):
            self.assertNotEqual(threading.get_ident(),caller)
            time.sleep(.65)  # Longer than the actual 0.5 s freshness limit.
            return {'presence':'PRESENT'},{}
        device.healthy=healthy
        with patch('hardware.analyze',side_effect=slow):
            self.assertEqual(device.analyze_with_feedback([],[],[],{}),
                             ({'presence':'PRESENT'},{}))
        self.assertGreater(len(checks),10)
        self.assertLess(max(b-a for a,b in zip(checks,checks[1:])),.5)

    def test_real_feedback_failure_still_aborts_analysis(self):
        device=self.device()
        def unhealthy(camera=True):
            raise RuntimeError('bridge feedback fault')
        def slow(*args,**kwargs):
            time.sleep(.05)
            return {},{}
        device.healthy=unhealthy
        with patch('hardware.analyze',side_effect=slow):
            with self.assertRaisesRegex(RuntimeError,'bridge feedback fault'):
                device.analyze_with_feedback([],[],[],{})

    def test_analysis_error_is_not_hidden(self):
        device=self.device()
        device.healthy=lambda camera=True: time.sleep(.001)
        with patch('hardware.analyze',side_effect=ValueError('bad frame')):
            with self.assertRaisesRegex(ValueError,'bad frame'):
                device.analyze_with_feedback([],[],[],{})


if __name__=='__main__':
    unittest.main(verbosity=2)
