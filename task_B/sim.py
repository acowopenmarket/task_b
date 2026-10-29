"""Deterministic adapters; never import ROS or open a device."""
import copy
import time
from types import SimpleNamespace
from core import validate
from estun_segmented.core import plan
from vision import tag


def demo_config(c):
    c=copy.deepcopy(c)
    c['observation_pose']=[.4,0,.5,-3.141592653589793,0,0]
    c['box'].update(xy_bounds=[.25,.55,-.15,.15],floor_z_abc=[0,0,0],rim_z_m=.12)
    c['safety'].update(clearance_m=.05,workspace_bounds=[.1,.7,-.4,.4,.205,.8])
    validate(c)
    return c


def present(x=.4,y=0,z=.06,top=.11):
    return dict(presence='PRESENT',localizable=True,top_valid=True,
                candidate_base_m=[x,y,z],z_top=top,reason='SIMULATED',top_quality={'method':'SIMULATED'})


class Vision:
    def __init__(self,sequence=None):
        self.sequence=list(sequence or [present(),present(),dict(presence='ABSENT'),dict(presence='ABSENT')])
        self.count=0
    def capture(self,deadline=None):
        if not self.sequence: raise TimeoutError('模拟输入耗尽')
        self.count+=1
        value=copy.deepcopy(self.sequence.pop(0))
        value.setdefault('localizable',False); value.setdefault('top_valid',False)
        value.setdefault('capture_id','sim-'+str(self.count)); value.setdefault('captured_monotonic',time.monotonic())
        return value


class Motion:
    def __init__(self,c,journal,fail_at=None):
        self.pose=list(c['observation_pose']); self.c=c; self.j=journal
        self.calls=[]; self.fail_at=fail_at
    def read_start(self): return SimpleNamespace(pose=tuple(self.pose))
    def move_to(self,p,stage='all',execute=False):
        self.calls.append((stage,list(p)))
        if self.fail_at==len(self.calls): raise RuntimeError('模拟通信/运动故障')
        from estun_segmented.core import Settings
        from estun_segmented.core import Completion,Feedback
        s=Settings(**self.c['motion'])
        for segment in plan(self.pose,p,s,stage):
            self.j.write('SIMULATED_SEGMENT',stage=segment.stage,target=segment.pose)
            # Exercise the real completion monitor with simulated controller feedback.
            sent=time.monotonic(); monitor=Completion(segment,sent,s)
            monitor.feed(Feedback(tuple(self.pose),(0,)*6,'AutoRunning',0,sent+.01))
            finished=False
            step=min(s.feedback_timeout/3,s.settle_seconds/max(1,s.settle_samples-1))
            for sample in range(max(10,s.settle_samples*3)):
                stamp=sent+.02+sample*step
                finished=monitor.feed(Feedback(segment.pose,(0,)*6,'AutoReady',0,stamp))
                if finished: break
            if not finished: raise TimeoutError('模拟反馈未通过原Completion检查')
            self.j.write('SIMULATED_FEEDBACK_ARRIVED',state='AutoReady',pose=segment.pose)
        self.pose=list(p)


class Gripper:
    def __init__(self,fail_at=None): self.calls=[]; self.fail_at=fail_at
    def action(self,close):
        self.calls.append(close)
        if self.fail_at==len(self.calls): raise TimeoutError('模拟夹爪动作超时')
