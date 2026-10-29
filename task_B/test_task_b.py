"""Offline invariant and fault-injection suite; never connects a device."""
import copy
import json
import math
from pathlib import Path
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import paths
from core import Task,compute_target,validate
from sim import demo_config,present,Vision,Motion,Gripper
from vision import analyze
from coordinate_transform.geometry import RobotConvention


class Log:
    def __init__(self): self.events=[]
    def write(self,event,**data): self.events.append(dict(event=event,**copy.deepcopy(data)))


def config():
    return demo_config(json.loads((paths.ROOT/'config.json').read_text(encoding='utf-8')))


def run(sequence=None,c=None,fail_motion=None,fail_gripper=None):
    c=c or config(); j=Log(); m=Motion(c,j,fail_motion); g=Gripper(fail_gripper)
    t=Task(c,m,g,Vision(sequence),j)
    return t.run(),j,m,g


def pair(status):
    return [copy.deepcopy(status),copy.deepcopy(status)]


class HeightTests(unittest.TestCase):
    def test_insert_less_equal_greater(self):
        for d in (.03,.044,.06):
            with self.subTest(d=d):
                o=present(z=.08,top=.08+d); o['capture_id']='h'
                t=compute_target(o,config())
                self.assertAlmostEqual(t['selected_grasp_base_m'][2],max(.08,.08+d-.044))
                self.assertEqual(t['selected_grasp_base_m'][:2],[.4,0])

    def test_both_limits_and_reference_error(self):
        o=present(z=.01,top=.06); o['capture_id']='h'
        t=compute_target(o,config())
        self.assertGreater(t['layer1_adjustment_m'],0)
        self.assertTrue(t['z_clamped']); self.assertEqual(t['flange_pose'][2],.213)
        self.assertAlmostEqual(t['final_reference_delta_from_candidate_m'][2],.213-.15384-.01)
        self.assertLess(t['original_flange_pose'][2],.213)

    def test_offsets_use_selected_orientation(self):
        for rpy in ([math.pi,0,0],[math.pi,0,math.pi/2],[2.8,.2,-.3]):
            c=config(); c['grasp_rpy_rad']=rpy
            c['flange_to_gripper_m']=[.01,.02,.15384]
            o=present(z=.15,top=.18); o['capture_id']='h'
            t=compute_target(o,c)
            expected=np.array(t['selected_grasp_base_m'])-RobotConvention().rotation(rpy)@c['flange_to_gripper_m']
            np.testing.assert_allclose(t['original_flange_pose'][:3],expected)

    def test_invalid_top_no_target(self):
        o=present(); o.update(capture_id='h',top_valid=False)
        with self.assertRaises(ValueError): compute_target(o,config())

    def test_five_positions_same_default_direction_without_classification(self):
        for x,y,name in [(.3,-.1,'corner_1'),(.5,-.1,'corner_2'),(.3,.1,'corner_3'),(.5,.1,'corner_4'),(.4,0,'center')]:
            o=present(x,y); o['capture_id']='h'; t=compute_target(o,config())
            self.assertNotIn('region',t); self.assertEqual(t['rpy_rad'],config()['grasp_rpy_rad'])

    def test_continuous_positions_not_limited_to_old_five_rectangles(self):
        c=config(); self.assertNotIn('regions',c)
        for x in np.linspace(.27,.53,7):
            for y in np.linspace(-.13,.13,7):
                o=present(float(x),float(y)); o['capture_id']='continuous'
                t=compute_target(o,c)
                self.assertEqual(t['selected_grasp_base_m'][:2],[float(x),float(y)])
                self.assertEqual(t['rpy_rad'],c['grasp_rpy_rad'])


class SequenceTests(unittest.TestCase):
    def test_full_flow_at_positions_previously_outside_five_rectangles(self):
        for x,y in [(.28,0),(.4,.12),(.52,0),(.4,-.12)]:
            r,j,m,g=run(pair(present(x,y))+pair(dict(presence='ABSENT')))
            self.assertTrue(r['success'],r)
            target=next(e for e in j.events if e['event']=='HEIGHT_AND_TARGET')
            self.assertEqual(target['candidate_base_m'][:2],[x,y])
            self.assertNotIn('region',target)
    def test_success_opens_and_finishes(self):
        r,j,m,g=run()
        self.assertTrue(r['success']); self.assertEqual(g.calls,[False,True,False])
        self.assertEqual(j.events[-1]['event'],'SUCCESS')
        self.assertEqual(r['success_reason'],'SOURCE_TARGET_ABSENT')

    def test_near_target_success_after_return(self):
        near=dict(present(),rgbd_candidate={'depth_m':.170})
        r,j,m,g=run(pair(present())+[near])
        self.assertTrue(r['success'],r)
        self.assertEqual(r['attempts'],1)
        self.assertEqual(r['success_reason'],'TARGET_CAMERA_DEPTH_BELOW_171MM')
        self.assertEqual(r['target_camera_depth_m'],.170)
        self.assertEqual(g.calls,[False,True,False])
        events=[e['event'] for e in j.events]
        self.assertLess(events.index('RESTORE_OBSERVATION_CLOSED'),events.index('SUCCESS'))

    def test_near_depth_boundary_and_invalid_values_do_not_succeed(self):
        c=config(); c['max_extra_retries']=0
        for depth in (.171,.172,0,-.1,float('nan'),float('inf'),None,'0.170',True):
            with self.subTest(depth=depth):
                review=dict(present(),rgbd_candidate={'depth_m':depth})
                r,j,m,g=run(pair(present())+pair(review),c)
                self.assertFalse(r['success'],r)
                self.assertEqual(j.events[-1]['event'],'RETRY_LIMIT')

    def test_single_near_burst_after_far_review_succeeds(self):
        far=dict(present(),rgbd_candidate={'depth_m':.2})
        near=dict(present(),rgbd_candidate={'depth_m':.170})
        r,j,m,g=run(pair(present())+[far,near])
        self.assertTrue(r['success'],r)
        self.assertEqual(sum(e['event']=='OBSERVATION' for e in j.events),4)

    def test_single_near_burst_after_unknown_succeeds(self):
        near=dict(present(),rgbd_candidate={'depth_m':.170})
        r,j,m,g=run(pair(present())+[dict(presence='UNKNOWN'),near])
        self.assertTrue(r['success'],r)
        self.assertEqual(sum(e['event']=='OBSERVATION' for e in j.events),4)

    def test_absent_still_requires_consecutive_bursts(self):
        r,j,m,g=run(pair(present())+[dict(presence='ABSENT')])
        self.assertFalse(r['success'],r)
        self.assertEqual(g.calls,[False,True])

    def test_recorded_review_returns_at_146mm_before_218mm(self):
        log=paths.ROOT/'runs/panel_20261029_173948_a7c0075c/program/events.jsonl'
        events=[json.loads(line) for line in log.read_text(encoding='utf-8').splitlines()]
        restore=next(i for i,e in enumerate(events) if e['event']=='RESTORE_OBSERVATION_CLOSED')
        reviews=[e for e in events[restore:] if e['event']=='OBSERVATION']
        # Retain real statuses and depths, but refresh IDs/times for an offline
        # state-machine replay; recorded captures must never drive hardware.
        for obs in reviews:
            for key in ('capture_id','captured_monotonic','event','time','monotonic'):
                obs.pop(key,None)
        c=config(); j=Log(); m=Motion(c,j); g=Gripper(); v=Vision(reviews)
        review=Task(c,m,g,v,j).observe()
        self.assertEqual(v.count,3)
        self.assertAlmostEqual(review['rgbd_candidate']['depth_m'],.146,places=5)
        self.assertEqual(len(v.sequence),2)
        self.assertEqual(m.calls,[])
        self.assertEqual(g.calls,[])

    def test_unknown_near_target_does_not_succeed(self):
        review=dict(presence='UNKNOWN',rgbd_candidate={'depth_m':.170})
        r,j,m,g=run(pair(present())+pair(review))
        self.assertFalse(r['success'],r)
        self.assertEqual(g.calls,[False,True])

    def test_near_target_before_grasp_still_requires_full_pick(self):
        near=dict(present(),rgbd_candidate={'depth_m':.170})
        r,j,m,g=run(pair(near)+pair(dict(presence='ABSENT')))
        self.assertTrue(r['success'],r)
        self.assertEqual(g.calls,[False,True,False])
        self.assertEqual(r['success_reason'],'SOURCE_TARGET_ABSENT')

    def test_retry_uses_new_coordinates_after_open(self):
        seq=pair(present())+pair(present())+pair(present(.42))+pair(dict(presence='ABSENT'))
        r,j,m,g=run(seq)
        self.assertTrue(r['success']); self.assertEqual(r['attempts'],2)
        targets=[e for e in j.events if e['event']=='HEIGHT_AND_TARGET']
        self.assertEqual([t['candidate_base_m'][0] for t in targets],[.4,.42])
        self.assertNotEqual(targets[0]['capture_id'],targets[1]['capture_id'])
        self.assertEqual(g.calls,[False,True,False,True,False])

    def test_retry_limit(self):
        r,j,m,g=run(pair(present())*6)
        self.assertFalse(r['success']); self.assertEqual(r['attempts'],3)
        self.assertEqual(g.calls,[False,True,False,True,False,True,False])
        self.assertEqual(j.events[-1]['event'],'RETRY_LIMIT')

    def test_unknown_then_absent(self):
        seq=pair(present())+[dict(presence='UNKNOWN',reason='camera_error')]+pair(dict(presence='ABSENT'))
        r,j,m,g=run(seq)
        self.assertTrue(r['success']); self.assertEqual(g.calls.count(True),1)

    def test_unknown_after_grasp_never_opens_or_retries(self):
        r,j,m,g=run(pair(present())+[dict(presence='UNKNOWN')]*3)
        self.assertFalse(r['success']); self.assertEqual(g.calls,[False,True])
        self.assertNotIn('SUCCESS',[e['event'] for e in j.events])

    def test_top_invalid_no_grasp_motion(self):
        o=present(); o['top_valid']=False
        r,j,m,g=run(pair(o))
        self.assertFalse(r['success']); self.assertEqual(g.calls,[False])
        self.assertNotIn('DESCEND',[e['event'] for e in j.events])

    def test_absent_before_first_pick_not_success(self):
        r,j,m,g=run(pair(dict(presence='ABSENT')))
        self.assertFalse(r['success']); self.assertEqual(g.calls,[False])

    def test_stale_and_reused_capture_abort(self):
        for patch in ({'captured_monotonic':time.monotonic()-100},{'capture_id':'same'}):
            seq=pair(dict(present(),**patch))
            r,j,m,g=run(seq)
            self.assertFalse(r['success']); self.assertEqual(g.calls,[False])

    def test_lift_before_return_and_arrive_before_close(self):
        r,j,m,g=run()
        events=[e['event'] for e in j.events]
        self.assertLess(events.index('DESCEND'),events.index('CLOSE_COMPLETED'))
        self.assertLess(events.index('LIFT_CLOSED'),events.index('RETURN_CLOSED'))
        descent=next(i for i,(s,p) in enumerate(m.calls) if s=='z' and p[2]<.5)
        self.assertEqual(m.calls[descent+1][0],'z')
        self.assertEqual(m.calls[descent+1][1][2],.5)
        self.assertEqual(m.calls[descent+2][0],'xy')

    def test_motion_fault_stops_all_following_actions(self):
        for failure in range(1,8):
            r,j,m,g=run(fail_motion=failure)
            self.assertFalse(r['success']); self.assertEqual(len(m.calls),failure)
            if failure<=4: self.assertNotIn(True,g.calls)
            self.assertEqual(j.events[-1]['event'],'ABORT')

    def test_arrival_failure_no_close(self):
        c=config(); j=Log(); m=Motion(c,j); g=Gripper()
        original=m.move_to
        def bad(p,stage='all',execute=False):
            original(p,stage,execute)
            if stage=='z': m.pose[2]+=.01
        m.move_to=bad
        r=Task(c,m,g,Vision(),j).run()
        self.assertFalse(r['success']); self.assertEqual(g.calls,[False])

    def test_gripper_fault_stops_movement(self):
        for fail,expected_moves in [(1,1),(2,4),(3,7)]:
            r,j,m,g=run(fail_gripper=fail)
            self.assertFalse(r['success']); self.assertEqual(len(m.calls),expected_moves)
            self.assertEqual(len(g.calls),fail)

    def test_unknown_timeout_no_motion(self):
        c=config(); j=Log(); m=Motion(c,j); g=Gripper()
        class Clock:
            def __init__(self): self.t=time.monotonic()
            def __call__(self): self.t+=1; return self.t
        c['vision']['timeout_s']=2
        t=Task(c,m,g,Vision([dict(presence='UNKNOWN')]*10),j,Clock())
        r=t.run(); self.assertFalse(r['success']); self.assertEqual(g.calls,[False])


def vision_fixture(object_position=(34,24),hole=False):
    c=config(); c['box'].update(xy_bounds=[-.32,.32,-.22,.22],floor_z_abc=[0,0,0],rim_z_m=.12,wall_margin_m=.001)
    info=dict(width=80,height=60,k=[100,0,40,0,100,30,0,0,1],d=[0]*5,distortion_model='plumb_bob',frame_id='camera')
    robot=RobotConvention()
    cal=dict(status='validated_real',heldout_consistency_verified=True,camera_info=info,camera_frame='camera',
             convention=robot.evidence(),T_flange_camera=np.eye(4).tolist())
    r=dict(camera_info=info,depth_camera_info=info,camera_frame='camera',
           robot_pose_at_capture=[0,0,1,math.pi,0,0],robot_fault=0,robot_state='AutoReady',
           image_stamp=1.,robot_stamp=1.,capture_sync_error_sec=.01,depth_encoding='32FC1')
    frames=np.ones((3,60,80),np.float32); empty=np.full((60,80,3),100,np.uint8)
    rgbs=np.stack([empty.copy() for _ in range(3)])
    if object_position:
        x,y=object_position; frames[:,y:y+18,x:x+18]=0 if hole else .94
        rgbs[:,y:y+18,x:x+18]=200
    return c,cal,[copy.deepcopy(r) for _ in range(3)],frames,rgbs,empty


class ConfigTests(unittest.TestCase):
    def test_missing_live_config_lists_fields(self):
        c=json.loads((paths.ROOT/'config.json').read_text(encoding='utf-8'))
        c['observation_pose']=None
        with self.assertRaisesRegex(ValueError,'observation_pose'):
            validate(c,True)

    def test_lower_flange_limit_rejected(self):
        c=config(); c['min_flange_z_m']=.204
        with self.assertRaises(ValueError): validate(c)

    def test_unsafe_observation_rejected(self):
        c=config(); c['observation_pose'][2]=.25
        with self.assertRaises(ValueError): validate(c)

    def test_extra_base_rectangles_are_optional(self):
        c=config(); c['box']['xy_bounds']=None; c['safety']['workspace_bounds']=None
        validate(c)
        o=present(); o['capture_id']='optional'
        a=compute_target(o,c); b=compute_target(o,config())
        self.assertEqual(a['flange_pose'],b['flange_pose'])

    def test_missing_optional_rectangles_are_accepted(self):
        c=config(); del c['box']['xy_bounds']; del c['safety']['workspace_bounds']
        validate(c)
        o=present(); o['capture_id']='optional'
        self.assertGreaterEqual(compute_target(o,c)['flange_pose'][2],.205)


class RGBDAdapterTests(unittest.TestCase):
    def evaluate(self,d,context=None):
        return analyze(d[3],d[4],d[2],d[1],d[0],source_context=context)

    def test_surface_depth_in_base_coordinates(self):
        r,_=self.evaluate(vision_fixture())
        self.assertTrue(r['localizable'],r)
        self.assertAlmostEqual(r['candidate_base_m'][2],.06,places=5)
        self.assertAlmostEqual(r['z_top'],.06,places=5)

    def test_target_positions_without_box_or_table(self):
        for position in [(14,14),(54,14),(14,34),(54,34),(34,24)]:
            d=vision_fixture(position)
            d[0]['box'].clear()
            r,_=self.evaluate(d)
            self.assertTrue(r['localizable'],r)

    def test_missing_depth_and_bad_sync_not_absent(self):
        d=vision_fixture();d[3][:]=0
        self.assertEqual(self.evaluate(d)[0]['presence'],'UNKNOWN')
        d=vision_fixture();d[2][0]['capture_sync_error_sec']=.2
        self.assertEqual(self.evaluate(d)[0]['presence'],'UNKNOWN')

    def test_source_cleared_with_local_depth_evidence(self):
        first,_=self.evaluate(vision_fixture())
        self.assertTrue(first['localizable'],first)
        review,_=self.evaluate(vision_fixture(None),first['source_context'])
        self.assertEqual(review['presence'],'ABSENT',review)

    def test_no_initial_target_is_not_task_success(self):
        r,_=self.evaluate(vision_fixture(None))
        self.assertEqual(r['presence'],'UNKNOWN')

    def test_full_task_with_rgbd_adapter(self):
        d=vision_fixture();first,_=self.evaluate(d)
        review,_=self.evaluate(vision_fixture(None),first['source_context'])
        first.pop('source_context',None)
        d[0]['safety']['workspace_bounds']=None
        d[0]['observation_pose']=d[2][0]['robot_pose_at_capture']
        result,j,m,g=run(pair(first)+pair(review),d[0])
        self.assertTrue(result['success'],result)

    def test_full_task_with_rgbd_near_target_review(self):
        d=vision_fixture(); first,_=self.evaluate(d)
        close=vision_fixture(); close[3][:,24:42,34:52]=.170
        review,_=self.evaluate(close,first['source_context'])
        self.assertEqual(review['presence'],'PRESENT',review)
        self.assertAlmostEqual(review['rgbd_candidate']['depth_m'],.170,places=5)
        first.pop('source_context',None)
        d[0]['safety']['workspace_bounds']=None
        d[0]['observation_pose']=d[2][0]['robot_pose_at_capture']
        result,j,m,g=run(pair(first)+[review],d[0])
        self.assertTrue(result['success'],result)
        self.assertEqual(result['success_reason'],'TARGET_CAMERA_DEPTH_BELOW_171MM')


class HardwareAdapterTests(unittest.TestCase):
    def camera_devices(self,duplicate=False):
        from hardware import Devices
        d=Devices(config(),Path('.'),Log(),False)
        d.children=[SimpleNamespace(poll=lambda:None)]
        d.buffers={k:[] for k in ('color','depth','ci','di')}
        ticks=[0]
        def healthy(camera=False):
            ticks[0]+=1
            if ticks[0]>=3:
                for k in d.buffers:d.buffers[k]=[object()]
        d.healthy=healthy
        d.node=SimpleNamespace(count_publishers=lambda topic:2 if duplicate else (1 if ticks[0]>=3 else 0))
        return d,ticks

    def test_camera_startup_waits_for_topics_and_images(self):
        d,ticks=self.camera_devices();d.wait_camera_ready()
        self.assertGreaterEqual(ticks[0],3)

    def test_camera_startup_rejects_duplicate_publishers(self):
        d,_=self.camera_devices(True)
        with self.assertRaisesRegex(RuntimeError,'重复'):d.wait_camera_ready()

    def test_camera_startup_timeout_is_not_success(self):
        d,_=self.camera_devices();d.healthy=lambda camera=False:None
        class Clock:
            def __init__(self):self.t=0
            def monotonic(self):self.t+=1;return self.t
        with patch('hardware.time',Clock()),self.assertRaises(TimeoutError):d.wait_camera_ready()

    def setup_devices(self,fail=False,never_complete=False):
        from hardware import Devices
        j=Log(); d=Devices(config(),Path('.'),j,True)
        d.healthy=lambda:None
        d.node=SimpleNamespace(read_start=lambda:None)
        class FakeSerialDriver:
            def __init__(self): self.sent=[]; self.queries=0
            def move(self,*args): self.sent.append(args)
            def query(self):
                self.queries+=1
                if fail and self.queries>1: raise OSError('串口断开')
                running=never_complete or 1<self.queries<4
                return SimpleNamespace(raw_angle=100 if running else 200,
                                       target_reached=not running,raw_speed=10 if running else 0)
        d.gripper=FakeSerialDriver()
        class Clock:
            def __init__(self): self.t=0
            def monotonic(self): self.t+=.1; return self.t
        return d,j,Clock()

    def test_gripper_waits_for_post_send_real_feedback(self):
        d,j,clock=self.setup_devices()
        with patch('hardware.time',clock): d.action(True)
        self.assertEqual(len(d.gripper.sent),1)
        self.assertGreaterEqual(d.gripper.queries,4)
        self.assertGreaterEqual(clock.t,32.67256359733385/20)
        self.assertEqual(j.events[-1]['event'],'GRIPPER_MOTOR_COMPLETE')

    def test_gripper_serial_failure_does_not_replay(self):
        d,j,clock=self.setup_devices(fail=True)
        with patch('hardware.time',clock),self.assertRaises(OSError): d.action(True)
        self.assertEqual(len(d.gripper.sent),1)
        self.assertNotIn('GRIPPER_MOTOR_COMPLETE',[e['event'] for e in j.events])

    def test_gripper_busy_until_timeout_is_failure(self):
        d,j,clock=self.setup_devices(never_complete=True)
        with patch('hardware.time',clock),self.assertRaises(TimeoutError): d.action(True)
        self.assertEqual(len(d.gripper.sent),1)


if __name__=='__main__': unittest.main(verbosity=2)
