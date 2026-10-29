import unittest
import numpy as np
from vision_center.grasp_candidate import compute_candidate, aligned_profiles
from vision_center.candidate_pipeline import add_base_coordinates
from coordinate_transform.geometry import RobotConvention
from configs import load,DEFAULT_CONFIG,PROJECT_ROOT


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.info=dict(width=160,height=120,k=[150.,0,80.,0,150.,60.,0,0,1],
                       d=[0.]*5,distortion_model='plumb_bob',frame_id='camera_color_optical_frame')
        self.frames=np.full((5,120,160),1000,np.uint16)
        self.frames[:,30:90,40:120]=800

    def compute(self,frames=None,rgb=None):
        return compute_candidate(self.frames if frames is None else frames,'16UC1',self.info,rgb=rgb)

    def assert_failed(self,result):
        self.assertFalse(result['success'])
        for key in ('pixel','depth_m','point_camera_m','point_base_m'): self.assertNotIn(key,result)

    def test_exact_bbox_center_and_all_enclosed_depth_pixels(self):
        result,debug=self.compute()
        self.assertTrue(result['success'])
        self.assertEqual(result['bbox_pixel_bounds'],[40,30,119,89])
        self.assertEqual(result['pixel'],[79.5,59.5])
        self.assertAlmostEqual(result['depth_m'],.8)
        self.assertEqual(result['valid_point_count'],4800)
        self.assertEqual(int(debug['region_valid_depth_mask'].sum()),4800)
        self.assertEqual(result['point_kind'],'bbox_center_region_depth_candidate')
        self.assertNotIn('contour_centroid_pixel',result)
        self.assertNotIn('local_depth',result)
        expected=[(79.5-80)/150*.8,(59.5-60)/150*.8,.8]
        np.testing.assert_allclose(result['point_camera_m'],expected)

    def test_region_includes_interior_not_only_boundary_or_center_roi(self):
        self.frames[:,32:88,42:118]=700
        self.frames[:,56:64,76:84]=600
        result,_=self.compute()
        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['depth_m'],.7)
        self.assertEqual(result['pixel'],[79.5,59.5])
        self.assertEqual(result['depth_m'],float(np.median(result['region_valid_depth_values_m'])))

    def test_missing_center_never_moves_xy(self):
        self.frames[:,45:75,65:95]=0
        result,_=self.compute()
        self.assertTrue(result['success'])
        self.assertEqual(result['pixel'],[79.5,59.5])
        self.assertAlmostEqual(result['depth_m'],.8)

    def test_background_center_never_moves_xy(self):
        self.frames[:,30:75,65:120]=1000
        result,_=self.compute()
        self.assertTrue(result['success'])
        self.assertEqual(result['pixel'],[79.5,59.5])
        self.assertAlmostEqual(result['depth_m'],.8)

    def test_zeros_nonfinite_table_depth_excluded_from_full_region(self):
        frames=self.frames.astype(np.float32)/1000
        frames[:,45:65,65:85]=np.nan
        frames[:,70:80,70:80]=0
        frames[:,50:60,90:100]=1.
        result,_=compute_candidate(frames,'32FC1',self.info)
        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['depth_m'],.8,places=6)
        self.assertTrue(np.isfinite(result['region_valid_depth_values_m']).all())
        self.assertLess(result['valid_point_count'],4800)
        self.assertEqual(result['pixel'],[79.5,59.5])

    def test_missing_all_depth_fails_without_coordinates(self):
        self.assert_failed(self.compute(np.zeros_like(self.frames))[0])

    def test_no_object_and_multiple_objects_fail(self):
        self.assert_failed(self.compute(np.full_like(self.frames,1000))[0])
        self.frames[:]=1000; self.frames[:,25:60,15:55]=800; self.frames[:,65:105,105:145]=850
        result,_=self.compute(); self.assert_failed(result)
        self.assertIn('MULTIPLE_OBJECTS',result['reason'])

    def test_temporal_instability_fails(self):
        self.frames[0,30:90,40:120]=780
        result,_=self.compute(); self.assert_failed(result)
        self.assertIn('TEMPORAL_REGION_DEPTH_UNSTABLE',result['reason'])

    def test_missing_region_in_one_frame_fails(self):
        self.frames[0,30:90,40:120]=0
        result,_=self.compute(); self.assert_failed(result)
        self.assertIn('INSUFFICIENT_VALID_REGION_DEPTH',result['reason'])

    def test_single_frame_cannot_claim_stability(self):
        result,_=self.compute(self.frames[-1]); self.assert_failed(result)

    def test_clipped_object_fails(self):
        self.frames[:,30:90,:80]=800
        result,_=self.compute(); self.assert_failed(result)
        self.assertIn('CLIPPED',result['reason'])

    def test_profile_equivalence_and_mismatch(self):
        profile=dict(self.info,d=[0.]*8,distortion_model='rational_polynomial')
        self.assertTrue(aligned_profiles(self.info,profile))
        profile['k']=[160.,0,80.,0,150.,60.,0,0,1]
        self.assertFalse(aligned_profiles(self.info,profile))

    def test_base_transform_uses_capture_pose_and_existing_handeye(self):
        result,_=self.compute(); robot=RobotConvention()
        cal=load(PROJECT_ROOT/'configs/calibration.example.json')
        cal['camera_info']=self.info; cal['T_flange_camera']=np.eye(4).tolist()
        # This positive test supplies synthetic validation; the public template
        # itself must remain unvalidated and cannot authorize real motion.
        cal.update(status='validated_real',heldout_consistency_verified=True)
        context=dict(camera_info=self.info,camera_frame=self.info['frame_id'],robot_fault=0,
            image_stamp=1.,robot_stamp=1.01,capture_sync_error_sec=.01,
            robot_pose_component_ranges=[0.]*6,robot_pose_at_capture=[.1,.2,.3,0,0,0])
        converted=add_base_coordinates(result,context,self.info,cal,robot,load(DEFAULT_CONFIG))
        self.assertTrue(converted['success'])
        np.testing.assert_allclose(converted['point_base_m'],np.array(result['point_camera_m'])+[.1,.2,.3])
        context['robot_stamp']=2.
        self.assert_failed(add_base_coordinates(result,context,self.info,cal,robot,load(DEFAULT_CONFIG)))

    def test_rgb_groups_fragments_but_keeps_bbox_center(self):
        rgb=np.full((120,160,3),180,np.uint8); rgb[30:90,40:120]=30
        self.frames[:]=1000
        self.frames[:,35:85,42:60]=800; self.frames[:,35:85,100:118]=800
        result,_=self.compute(rgb=rgb)
        self.assertTrue(result['success'])
        self.assertEqual(result['segmentation']['raw_depth_region_count'],2)
        self.assertEqual(result['pixel'],[79.5,59.5])
        self.assertAlmostEqual(result['depth_m'],.8)

    def test_rgb_does_not_merge_two_separated_objects(self):
        rgb=np.full((120,160,3),180,np.uint8); self.frames[:]=1000
        for y0,y1,x0,x1 in [(25,60,15,55),(65,105,105,145)]:
            self.frames[:,y0:y1,x0:x1]=800; rgb[y0:y1,x0:x1]=30
        result,_=self.compute(rgb=rgb)
        self.assert_failed(result); self.assertIn('MULTIPLE_OBJECTS',result['reason'])

    def test_partial_border_region_is_recorded_and_excluded(self):
        rgb=np.full((120,160,3),180,np.uint8)
        rgb[30:90,40:120]=30; rgb[20:80,145:160]=30
        self.frames[:,20:80,145:160]=800
        result,_=self.compute(rgb=rgb)
        self.assertTrue(result['success'])
        self.assertIn('OBJECT_REGION_CLIPPED',[r['decision'] for r in result['segmentation']['regions']])
