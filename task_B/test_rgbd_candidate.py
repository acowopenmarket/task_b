"""Offline tests of actual RGB-D localization; no robot dependencies/devices."""
import json
from pathlib import Path
import unittest
import cv2
import numpy as np
from rgbd_candidate import locate


class RGBDTests(unittest.TestCase):
    def scene(self,x=34,y=24):
        rgb=np.full((80,100,3),100,np.uint8)
        d=np.ones((3,80,100),np.float32)
        rgb[y:y+20,x:x+25]=[30,80,210]
        d[:,y:y+20,x:x+25]=.8
        info=dict(width=100,height=80,k=[100,0,50,0,100,40,0,0,1])
        return d,rgb,info

    def test_positions_and_depth_not_table_height(self):
        for x,y in [(5,5),(68,5),(5,52),(68,52),(34,24),(0,24)]:
            with self.subTest(x=x,y=y):
                d,rgb,info=self.scene(x,y)
                # Grooves/uneven support do not change the target depth.
                d[:,:,::7]=1.04
                d[:,y:y+20,x:x+25]=.8
                r,_,reason=locate(d,'32FC1',info,rgb,{})
                self.assertIsNotNone(r,reason)
                self.assertAlmostEqual(r['depth_m'],.8,places=5)
                self.assertLess(abs(r['pixel'][0]-(x+12)),3)
                self.assertLess(abs(r['pixel'][1]-(y+9.5)),3)

    def test_uniform_rgb_does_not_validate_depth_blob(self):
        d,rgb,info=self.scene();rgb[:]=100
        r,_,_=locate(d,'32FC1',info,rgb,{})
        self.assertIsNone(r)

    def test_target_holes_and_invalid_background(self):
        d,rgb,info=self.scene();d[:,29:35,40:48]=0;d[:,:4]=0
        r,_,reason=locate(d,'32FC1',info,rgb,{})
        self.assertIsNotNone(r,reason)
        self.assertAlmostEqual(r['depth_m'],.8,places=5)

    def test_temporal_depth_change_rejected(self):
        d,rgb,info=self.scene();d[0,24:44,34:59]=.75
        r,_,reason=locate(d,'32FC1',info,rgb,{})
        self.assertIsNone(r);self.assertEqual(reason,'TARGET_DEPTH_UNSTABLE')

    def test_recorded_small_low_contrast_target_stays_rejected_with_diagnostics(self):
        root=Path(__file__).parent
        p=root/'runs/20261028_151622_434/capture_c8bc08f4-f2db-40a7-aa0f-26c357ff5534'
        rec=json.loads((p/'records.json').read_text())[-1]
        cfg=json.loads((root/'config.json').read_text())['vision']
        diagnostics={}
        result,_,reason=locate(np.load(p/'depth.npy'),rec['depth_encoding'],rec['camera_info'],
            np.load(p/'rgb.npy')[-1],cfg,diagnostics=diagnostics)
        self.assertIsNone(result)
        self.assertEqual(reason,'NO_MATCHING_RGBD_OBJECT')
        self.assertEqual(diagnostics['required_rgb_depth_iou'],.6)
        self.assertEqual(diagnostics['failure_detail'],'RGB_DEPTH_IOU_BELOW_THRESHOLD')
        targets=[c for c in diagnostics['candidates'] if
            330<c['bbox'][0]<380 and 190<c['bbox'][1]<230]
        self.assertTrue(targets)
        for c in targets:
            self.assertLess(c['rgb_depth_iou'],.2)
            self.assertEqual(c['rejected_by'],'RGB_DEPTH_IOU_BELOW_THRESHOLD')
        self.assertTrue(all(c['rejected_by'] is not None for c in diagnostics['candidates']))

    def test_curved_multicolour_surface_spans_multiple_depth_bands(self):
        y,x=np.indices((120,160))
        radius=((x-80)/28)**2+((y-60)/23)**2
        mask=radius<=1
        d=np.ones((3,120,160),np.float32)
        d[:,mask]=(.75+.07*radius[mask]).astype(np.float32)
        rgb=np.full((120,160,3),100,np.uint8)
        rgb[mask & (y<60)]=[30,60,130]
        rgb[mask & (y>=60)]=[20,180,160]
        info=dict(width=160,height=120,k=[160,0,80,0,160,60,0,0,1])
        r,debug,reason=locate(d,'32FC1',info,rgb,{})
        self.assertIsNotNone(r,reason)
        np.testing.assert_allclose(r['pixel'],[80,60],atol=2)
        self.assertGreater(np.count_nonzero(debug['object_mask'] & mask)/mask.sum(),.9)
        self.assertGreater(r['rgb_depth_iou'],.75)

    def test_recorded_curved_object_and_consecutive_source_context(self):
        from vision import analyze
        root=Path(__file__).parent
        c=json.loads((root/'config.json').read_text())
        calibration=json.loads((root/c['hardware']['calibration_path']).read_text())
        context=None
        captures=[]
        for run in ('20261028_144818_413','20261028_144858_804','20261028_145200_323'):
            captures.extend(sorted((root/'runs'/run).glob('capture_*'),
                key=lambda p:json.loads((p/'records.json').read_text())[-1]['image_stamp']))
        self.assertGreaterEqual(len(captures),12)
        for p in captures:
            with self.subTest(capture=p.name):
                records=json.loads((p/'records.json').read_text())
                result,debug=analyze(np.load(p/'depth.npy'),np.load(p/'rgb.npy'),
                    records,calibration,c,source_context=context)
                self.assertEqual(result['presence'],'PRESENT',result)
                self.assertTrue(result['localizable'],result)
                self.assertTrue(result['top_valid'],result)
                # Entire curved target in these saved images, not the large
                # striped background that the old bands sometimes selected.
                np.testing.assert_allclose(result['pixel'],[326.5,243.5],atol=10)
                self.assertTrue(.30<result['rgbd_candidate']['depth_m']<.35)
                self.assertLess(np.mean(debug['object_mask']),.08)
                context=result['source_context']

    def test_real_captures_select_object_not_box_or_background(self):
        for folder in ['restart_20261027_170832','site_paper_validation_20261027']:
            with self.subTest(folder=folder):
                root=Path(__file__).parent/'runs'/folder/'fresh_preview'
                p=next(root.glob('capture_*'))
                rec=json.loads((p/'records.json').read_text())[-1]
                rgb=np.load(p/'rgb.npy')[-1]
                r,debug,reason=locate(np.load(p/'depth.npy'),rec['depth_encoding'],rec['camera_info'],rgb,{})
                self.assertIsNotNone(r,reason)
                self.assertGreater(r['rgb_depth_iou'],.85)
                self.assertTrue(.16<r['depth_m']<.19)
                # Manual RGB annotation of these two saved targets.
                expected=(325,259) if folder.startswith('restart') else (296,192)
                np.testing.assert_allclose(r['pixel'],expected,atol=5)
                self.assertLess(np.mean(debug['object_mask']),.3)


if __name__=='__main__':unittest.main(verbosity=2)
