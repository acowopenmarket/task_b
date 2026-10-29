"""Offline timestamp matching regressions; never connect devices."""
from collections import deque
from types import SimpleNamespace
import unittest
from capture_sync import match_buffered,timestamp


def entry(stamp,arrival=10.):
    sec=int(stamp)
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9))))
    return msg,arrival


class SyncTests(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(max_sync_s=.05,max_image_age_s=.5,max_robot_age_s=.2)
        self.keys=('depth','ci','di')

    def buffers(self):
        return {k:deque([entry(100.)]) for k in ('color',)+self.keys}

    def test_delayed_companion_matches_previous_color_not_only_newest(self):
        b=self.buffers(); b['color'].append(entry(100.066))
        bundle,_=match_buffered(b,self.keys,set(),10.1,self.cfg)
        self.assertIsNotNone(bundle)
        self.assertEqual(timestamp(bundle[0]),100.)

    def test_newest_complete_bundle_selected_and_not_reused(self):
        b=self.buffers()
        for q in b.values(): q.append(entry(100.066))
        bundle,_=match_buffered(b,self.keys,{100.},10.1,self.cfg)
        self.assertAlmostEqual(timestamp(bundle[0]),100.066)
        self.assertIsNone(match_buffered(b,self.keys,{100.,100.066},10.1,self.cfg)[0])
        self.assertIsNone(match_buffered(b,self.keys,{100.066},10.1,self.cfg)[0])

    def test_real_timestamp_mismatch_still_rejected(self):
        b=self.buffers();b['depth']=deque([entry(100.1)])
        bundle,diag=match_buffered(b,self.keys,set(),10.1,self.cfg)
        self.assertIsNone(bundle);self.assertGreater(diag['span_s'],.05)

    def test_stale_image_and_stale_robot_still_rejected(self):
        b=self.buffers()
        self.assertIsNone(match_buffered(b,self.keys,set(),10.6,self.cfg)[0])
        b['robot']=deque([entry(100.,9.8)])
        self.assertIsNone(match_buffered(b,self.keys+('robot',),set(),10.1,self.cfg)[0])

    def test_camera_only_needs_no_robot_and_missing_intrinsics_not_accepted(self):
        b=self.buffers()
        self.assertIsNotNone(match_buffered(b,self.keys,set(),10.1,self.cfg)[0])
        self.assertIsNone(match_buffered(b,self.keys+('robot',),set(),10.1,self.cfg)[0])
        b['ci'].clear()
        self.assertIsNone(match_buffered(b,self.keys,set(),10.1,self.cfg)[0])


if __name__=='__main__':unittest.main(verbosity=2)
