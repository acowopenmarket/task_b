import copy
import unittest
from record_observation import select_stable


class RecordTests(unittest.TestCase):
    def samples(self):
        return [dict(received=i*.05,feedback=dict(end_positions=[.3,.1,.42,-3.07,0,-1.57],
                    fault_flag=0,state='PowerOff')) for i in range(21)]
    def select(self,s): return select_stable(s,.8,.0005,.00175,.5)
    def test_poweroff_can_record_but_does_not_require_enabling(self):
        s=self.samples(); self.assertEqual(self.select(s),s[-1]['feedback']['end_positions'])
    def test_running_rejected(self):
        s=self.samples(); s[-1]['feedback']['state']='AutoRunning'; self.assertIsNone(self.select(s))
    def test_fault_rejected(self):
        s=self.samples(); s[-1]['feedback']['fault_flag']=1; self.assertIsNone(self.select(s))
    def test_position_drift_rejected(self):
        s=self.samples(); s[-1]['feedback']['end_positions'][0]+=.002; self.assertIsNone(self.select(s))
    def test_short_or_interrupted_stream_rejected(self):
        s=self.samples(); self.assertIsNone(self.select(s[:9]))
        s[-1]['received']+=1; self.assertIsNone(self.select(s))


if __name__=='__main__': unittest.main(verbosity=2)
