import ast
import json
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from coordinate_transform.geometry import (RobotConvention, inverse, camera_to_base, target_components,
                       flange_to_gripper, transform, select_grasp_point)
from vision_center.adapters import medical_center
from handeye_calibration.calibration import solve, detect_board, board_points
from coordinate_transform.target import plan_observation

from configs import DEFAULT_CONFIG, PROJECT_ROOT
ROOT=PROJECT_ROOT


def synthetic_samples(robot):
    cfg=json.loads(DEFAULT_CONFIG.read_text())
    rng=np.random.default_rng(20261023)
    x=robot.pose_matrix([.04,-.03,.06,.12,-.18,.22])
    board=robot.pose_matrix([.35,.1,.95,.03,.05,-.08])
    info=dict(width=640,height=480,k=[461.,0.,319.,0.,461.,239.,0.,0.,1.],
              d=[0.]*5,distortion_model='plumb_bob',frame_id=cfg['camera_frame'])
    samples=[]
    for i in range(25):
        pose=np.r_[np.array([.3,.1,.3])+rng.uniform(-.08,.08,3),rng.uniform(-.55,.55,3)]
        ct=inverse(robot.pose_matrix(pose)@x)@board
        samples.append(dict(id=f'sample_{i:03}',session_id='SYNTHETIC_DO_NOT_USE_ON_ROBOT',
            role='train' if i<20 else 'validation',data_kind='synthetic',board=cfg['board'],
            camera_frame=cfg['camera_frame'],camera_info=info,convention=robot.evidence(),
            robot_pose=pose.tolist(),robot_fault=0,origin_confirmed=True,
            reprojection_rms_px=0.,T_camera_board=ct.tolist()))
    return cfg,samples,x,board


class ReadOnlyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.robot=RobotConvention()

    def test_official_fixed_xyz_example(self):
        rx,ry,rz=np.radians([20,30,45])
        cx,sx=math.cos(rx),math.sin(rx); cy,sy=math.cos(ry),math.sin(ry); cz,sz=math.cos(rz),math.sin(rz)
        rxm=np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
        rym=np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
        rzm=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
        np.testing.assert_allclose(self.robot.rotation([rx,ry,rz]),rzm@rym@rxm,atol=1e-14)

    def test_inverse_chain(self):
        t=self.robot.pose_matrix([.1,.2,.3,.4,.5,.6])
        np.testing.assert_allclose(t@inverse(t),np.eye(4),atol=1e-14)

    def test_camera_point_known_rotation(self):
        a=self.robot.pose_matrix([1.,2.,3.,0.,0.,math.pi/2])
        x=self.robot.pose_matrix([.1,0.,0.,0.,0.,0.])
        np.testing.assert_allclose(camera_to_base([.2,0.,.5],a,x),[1.,2.3,3.5],atol=1e-14)

    def test_compensation_plus_z(self):
        r=target_components([.4,.1,.5],[0,0,0],self.robot)
        np.testing.assert_allclose(r['flange_pose'][:3],[.4,.1,.34616],atol=1e-14)
        self.assertLess(r['compensation_error_m'],1e-14)

    def test_compensation_rotated_grasp(self):
        r=target_components([.4,.1,.5],[0,math.pi/2,0],self.robot)
        np.testing.assert_allclose(r['flange_pose'][:3],[.24616,.1,.5],atol=1e-14)

    def test_below_z_outputs_clamped_pose_and_real_point_error(self):
        r=target_components([.1,.1,.3],[0,0,0],self.robot)
        self.assertTrue(r['z_limit_ok']); self.assertTrue(r['z_clamped'])
        self.assertFalse(r['original_z_limit_ok'])
        self.assertAlmostEqual(r['original_flange_pose'][2],.14616)
        self.assertAlmostEqual(r['flange_pose'][2],.213)
        np.testing.assert_allclose(r['flange_pose'][:2],[.1,.1])
        np.testing.assert_allclose(r['flange_pose'][3:],[0,0,0])
        self.assertEqual(r['robotmove_preview']['points'][0]['positions'],r['flange_pose'])
        self.assertAlmostEqual(r['z_adjustment_m'],.06684)
        np.testing.assert_allclose(r['gripper_point_error_base_m'],[0,0,.06684],atol=1e-14)
        self.assertAlmostEqual(r['compensation_error_m'],.06684)
        self.assertFalse(r['executable'])

    def test_exact_z_limit(self):
        r=target_components([.1,.1,.213],[0,0,0],self.robot,offset=[0,0,0])
        self.assertTrue(r['z_limit_ok'])
        self.assertFalse(r['z_clamped']); self.assertEqual(r['z_adjustment_m'],0.)

    def test_above_limit_pose_unchanged(self):
        r=target_components([.1,.2,.5],[-math.pi,0,0],self.robot)
        self.assertEqual(r['flange_pose'],r['original_flange_pose'])
        self.assertFalse(r['z_clamped'])
        self.assertLess(r['compensation_error_m'],1e-14)

    def test_configured_higher_limit_is_applied(self):
        r=target_components([.1,.2,.1],[-math.pi,0,0],self.robot,min_flange_z=.3)
        self.assertAlmostEqual(r['flange_pose'][2],.3)
        self.assertTrue(r['z_clamped'])

    def test_cannot_lower_limit(self):
        for limit in (.2,.201,.204,.205,.212):
            with self.assertRaises(ValueError):
                target_components([0,0,.4],[0,0,0],self.robot,min_flange_z=limit)

    def test_invalid_depth_and_nan(self):
        for p in ([0,0,0],[0,0,-1],[0,float('nan'),1]):
            with self.assertRaises(ValueError): camera_to_base(p,np.eye(4),np.eye(4))

    def test_transform_reflection_rejected(self):
        t=np.eye(4); t[0,0]=-1
        with self.assertRaises(ValueError): transform(t)

    def test_medical_mm_to_m(self):
        np.testing.assert_allclose(medical_center({'position_xyz':[100.,-20.,500.]}),[.1,-.02,.5])
        with self.assertRaises(ValueError): medical_center({'position_3d':[0,0,0]})

    def test_euler_wrap_and_gimbal(self):
        for rpy in ([0,math.pi/2,.3],[.5,-math.pi/2,-.6],[math.pi-.01,.3,-math.pi+.02]):
            p=self.robot.components([0,0,.4],rpy,[3.1,1.5,3.1])
            np.testing.assert_allclose(self.robot.rotation(p[3:]),self.robot.rotation(rpy),atol=1e-8)

    def test_robotmove_layout_reused(self):
        p=self.robot.robotmove_preview([.1,.2,.4,0,0,0])
        self.assertEqual(p['joint_names'],['x','y','z','rx','ry','rz'])
        self.assertEqual(len(p['points']),1); self.assertEqual(len(p['points'][0]['positions']),6)
        self.assertEqual(len(p['points'][0]['velocities']),2)

    def test_solve_synthetic_and_holdout(self):
        cfg,samples,x,_=synthetic_samples(self.robot); result=solve(samples,cfg,self.robot)
        np.testing.assert_allclose(result['T_flange_camera'],x,atol=1e-10)
        self.assertLess(result['validation']['all_corners_max_m'],1e-10)
        self.assertEqual(result['status'],'simulation_only'); self.assertFalse(result['real_accuracy_verified'])

    def test_holdout_cannot_tune_solution(self):
        cfg,samples,x,_=synthetic_samples(self.robot)
        for s in samples[20:]: s['T_camera_board'][0][3]+=.05
        result=solve(samples,cfg,self.robot)
        np.testing.assert_allclose(result['T_flange_camera'],x,atol=1e-10)
        self.assertGreater(result['validation']['all_corners_max_m'],.04)

    def test_real_bad_holdout_rejected(self):
        cfg,samples,_,_=synthetic_samples(self.robot)
        for s in samples: s['data_kind']='real_checkerboard'
        for s in samples[20:]: s['T_camera_board'][0][3]+=.05
        self.assertEqual(solve(samples,cfg,self.robot)['status'],'failed_validation')

    def test_degenerate_no_rotation_rejected(self):
        cfg,samples,_,_=synthetic_samples(self.robot)
        for s in samples: s['robot_pose'][3:]=[0.,0.,0.]
        with self.assertRaises(ValueError): solve(samples,cfg,self.robot)

    def test_missing_holdout_rejected(self):
        cfg,samples,_,_=synthetic_samples(self.robot)
        with self.assertRaises(ValueError): solve(samples[:20],cfg,self.robot)

    def test_mixed_profile_rejected(self):
        cfg,samples,_,_=synthetic_samples(self.robot)
        samples[1]['camera_info']=dict(samples[1]['camera_info'],width=1280)
        with self.assertRaises(ValueError): solve(samples,cfg,self.robot)

    def test_synthetic_not_accepted_for_real_target(self):
        cfg,samples,_,_=synthetic_samples(self.robot); cal=solve(samples,cfg,self.robot)
        obs=dict(robot_fault=0,camera_frame=cfg['camera_frame'],camera_info=cal['camera_info'],
                 convention=self.robot.evidence(),robot_stamp=1.,image_stamp=1.,capture_sync_error_sec=0.,
                 point_camera_m=[0,0,.5],robot_pose_at_capture=samples[0]['robot_pose'])
        grasp=dict(convention=self.robot.evidence(),operator_confirmed_for_calculation=True,grasp_rpy_rad=[0,0,0])
        with self.assertRaises(ValueError): plan_observation(obs,cal,grasp,cfg,self.robot)
        result=plan_observation(obs,cal,grasp,cfg,self.robot,True,point_is_grasp_center=True)
        self.assertFalse(result['eligible_for_motion_review']); self.assertIsNone(result['robotmove_preview'])

    def test_requested_downward_orientation(self):
        r=target_components([.4,.1,.08],[-math.pi,0,0],self.robot)
        np.testing.assert_allclose(r['flange_pose'],[.4,.1,.23384,-math.pi,0,0],atol=1e-14)
        self.assertTrue(r['z_limit_ok']); self.assertFalse(r['executable'])

    def test_visible_surface_needs_height_confirmation(self):
        with self.assertRaises(ValueError): select_grasp_point([.4,.1,.1])

    def test_top_surface_grasp_preserves_measured_height(self):
        top=[.32338566,.23788105,.10595149]
        selected,metadata=select_grasp_point(top,use_top_surface_center=True)
        np.testing.assert_allclose(selected,top)
        self.assertEqual(metadata['mode'],'measured_top_surface_center')
        self.assertEqual(metadata['height_difference_m'],0.)
        result=target_components(selected,[-math.pi,0,0],self.robot)
        self.assertAlmostEqual(result['flange_pose'][2],.25979149)
        self.assertFalse(result['executable'])

    def test_top_surface_policy_rejects_conflicting_height(self):
        with self.assertRaises(ValueError):
            select_grasp_point([.4,.1,.1],grasp_height_base_z_m=.05,use_top_surface_center=True)
        with self.assertRaises(ValueError):
            select_grasp_point([.4,.1,.1],point_confirmed=True,use_top_surface_center=True)

    def test_top_surface_pipeline_requires_matching_point_semantics(self):
        cfg,samples,_,_=synthetic_samples(self.robot); cal=solve(samples,cfg,self.robot)
        obs=dict(robot_fault=0,camera_frame=cfg['camera_frame'],camera_info=cal['camera_info'],
                 convention=self.robot.evidence(),robot_stamp=1.,image_stamp=1.,capture_sync_error_sec=0.,
                 point_camera_m=[0,0,.5],robot_pose_at_capture=samples[0]['robot_pose'])
        grasp=dict(convention=self.robot.evidence(),operator_confirmed_for_calculation=True,grasp_rpy_rad=[-math.pi,0,0])
        with self.assertRaisesRegex(ValueError,'TOP_SURFACE_CENTER_REQUIRED'):
            plan_observation(obs,cal,grasp,cfg,self.robot,True,use_top_surface_center=True)
        obs['point_kind']='top_surface_center'
        result=plan_observation(obs,cal,grasp,cfg,self.robot,True,use_top_surface_center=True)
        np.testing.assert_allclose(result['selected_grasp_point_base_m'],result['point_base_m'])
        self.assertFalse(result['motion_sent'])

    def test_default_top_surface_policy_uses_measured_point(self):
        cfg,samples,_,_=synthetic_samples(self.robot); cal=solve(samples,cfg,self.robot)
        cfg['grasp_point_selection']='measured_top_surface_center'
        obs=dict(robot_fault=0,camera_frame=cfg['camera_frame'],camera_info=cal['camera_info'],
                 convention=self.robot.evidence(),robot_stamp=1.,image_stamp=1.,capture_sync_error_sec=0.,
                 point_camera_m=[0,0,.5],robot_pose_at_capture=samples[0]['robot_pose'])
        grasp=dict(convention=self.robot.evidence(),operator_confirmed_for_calculation=True,grasp_rpy_rad=[-math.pi,0,0])
        with self.assertRaisesRegex(ValueError,'TOP_SURFACE_CENTER_REQUIRED'):
            plan_observation(obs,cal,grasp,cfg,self.robot,True)
        obs['point_kind']='top_surface_center'
        result=plan_observation(obs,cal,grasp,cfg,self.robot,True)
        np.testing.assert_allclose(result['selected_grasp_point_base_m'],result['point_base_m'])
        self.assertEqual(result['grasp_point_selection']['mode'],'measured_top_surface_center')
        self.assertFalse(result['motion_sent'])

    def test_top_surface_still_obeys_flange_z_limit(self):
        selected,_=select_grasp_point([.4,.1,.03],use_top_surface_center=True)
        result=target_components(selected,[-math.pi,0,0],self.robot)
        self.assertTrue(result['z_limit_ok']); self.assertTrue(result['z_clamped'])
        self.assertAlmostEqual(result['flange_pose'][2],.213)
        self.assertIsNotNone(result['robotmove_preview'])

    def test_selected_grip_height_is_not_surface_height(self):
        selected,metadata=select_grasp_point([.4,.1,.12],grasp_height_base_z_m=.08)
        np.testing.assert_allclose(selected,[.4,.1,.08])
        self.assertAlmostEqual(metadata['height_difference_m'],-.04)
        self.assertFalse(metadata['is_flange_z_clamp'])

    def test_selected_height_low_flange_is_clamped(self):
        selected,_=select_grasp_point([.4,.1,.12],grasp_height_base_z_m=.03)
        r=target_components(selected,[-math.pi,0,0],self.robot)
        self.assertTrue(r['z_limit_ok']); self.assertAlmostEqual(r['flange_pose'][2],.213)
        self.assertAlmostEqual(r['original_flange_pose'][2],.18384)
        self.assertIsNotNone(r['robotmove_preview'])

    def test_provisional_offset_blocks_motion_review(self):
        cfg,samples,_,_=synthetic_samples(self.robot)
        cal=solve(samples,cfg,self.robot)
        # Exercise the policy gate separately with a real-status-shaped fixture.
        # This fixture is never saved or presented as a real calibration result.
        cal.update(status='validated_real',heldout_consistency_verified=True)
        obs=dict(robot_fault=0,camera_frame=cfg['camera_frame'],camera_info=cal['camera_info'],
                 convention=self.robot.evidence(),robot_stamp=1.,image_stamp=1.,capture_sync_error_sec=0.,
                 point_camera_m=[0,0,.5],robot_pose_at_capture=samples[0]['robot_pose'])
        grasp=dict(convention=self.robot.evidence(),operator_confirmed_for_calculation=True,
                   grasp_rpy_rad=[-math.pi,0,0],gripper_offset_physically_verified=False)
        result=plan_observation(obs,cal,grasp,cfg,self.robot,point_is_grasp_center=True)
        self.assertTrue(result['calculation_uses_provisional_gripper_offset'])
        self.assertFalse(result['eligible_for_motion_review']); self.assertFalse(result['motion_sent'])

    def test_no_motion_publishers_or_sockets(self):
        for file in ('handeye_calibration/capture.py','handeye_calibration/cli.py','coordinate_transform/geometry.py','coordinate_transform/target.py','handeye_calibration/calibration.py'):
            tree=ast.parse((ROOT/file).read_text())
            calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
            self.assertNotIn('create_publisher',calls); self.assertNotIn('sendto',calls)
            self.assertNotIn('send_grab',calls)

    def test_checkerboard_image_detection(self):
        # Rasterized real pattern geometry exercises detection + PnP, still SYNTHETIC.
        cfg=json.loads(DEFAULT_CONFIG.read_text()); cell=40
        board=np.ones((9*cell,12*cell),np.uint8)*255
        for y in range(9):
            for x in range(12):
                if (x+y)%2==0: board[y*cell:(y+1)*cell,x*cell:(x+1)*cell]=0
        image=np.ones((480,640),np.uint8)*200; image[60:420,80:560]=board
        info=dict(width=640,height=480,k=[800.,0,320.,0,800.,240.,0,0,1.],d=[0.]*5,distortion_model='plumb_bob')
        t,corners,rms=detect_board(cv2.cvtColor(image,cv2.COLOR_GRAY2BGR),info,cfg['board'])
        self.assertEqual(len(corners),88); self.assertLess(rms,.1); self.assertAlmostEqual(t[2,3],.4,delta=.002)


def export_simulation(directory):
    out=Path(directory); out.mkdir(parents=True,exist_ok=True); robot=RobotConvention()
    cfg,samples,x,board=synthetic_samples(robot)
    for s in samples: (out/(s['id']+'.json')).write_text(json.dumps(s,indent=2))
    result=solve(samples,cfg,robot)
    (out/'synthetic_calibration.json').write_text(json.dumps(result,indent=2))
    (out/'ground_truth.json').write_text(json.dumps(dict(data_kind='synthetic',T_flange_camera=x.tolist(),T_base_board=board.tolist()),indent=2))


if __name__=='__main__': unittest.main(verbosity=2)
