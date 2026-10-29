"""Full-image RGB-D localization, coordinate conversion and target-only review."""
import time
import uuid
import cv2
import numpy as np
import paths
from rgbd_candidate import locate, depth_metres, rays_for
from vision_center.candidate_pipeline import add_base_coordinates
from coordinate_transform.geometry import RobotConvention
from coordinate_transform.pose_ranges import pose_component_ranges


def unknown(reason,**data):
    return dict(presence='UNKNOWN',localizable=False,top_valid=False,reason=reason,**data)


def supported(points,mask,span):
    z=points[...,2].astype(np.float32)
    support=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8))>0
    lo=cv2.erode(np.where(mask,z,1e6),np.ones((3,3),np.uint8))
    hi=cv2.dilate(np.where(mask,z,-1e6),np.ones((3,3),np.uint8))
    return support&((hi-lo)<=span)


def analyze(raw_frames,rgbs,records,calibration,c,empty_rgb=None,source_context=None):
    """A failed localization is never directly converted into ABSENT."""
    cfg=c['vision'];robot=RobotConvention();debug={}
    quality=dict(detector='rgb_depth_components_with_rgb_consistency',
                 scope='full_camera_image',uses_box_detector=False,uses_colour_threshold=False)
    try:
        if len(raw_frames)<cfg['frames'] or len(raw_frames)!=len(records) or len(rgbs)!=len(records):
            raise ValueError('INSUFFICIENT_PAIRED_FRAMES')
        ranges=pose_component_ranges([r['robot_pose_at_capture'] for r in records])
        transforms=[]
        for rgb,r in zip(rgbs,records):
            info=r['camera_info']
            if rgb.shape!=(info['height'],info['width'],3) or rgb.dtype!=np.uint8:
                raise ValueError('RGB_IMAGE_INVALID')
            if np.mean(np.max(rgb,axis=-1)<10)>.9 or np.mean(np.min(rgb,axis=-1)>250)>.9:
                raise ValueError('RGB_BLACKOUT_OR_OVEREXPOSURE')
            checked=add_base_coordinates(dict(success=True,point_camera_m=[0,0,1]),
                dict(r,robot_pose_component_ranges=ranges),r['depth_camera_info'],calibration,robot,
                dict(max_sync_sec=cfg['max_sync_s'],settle_translation_m=cfg['settle_translation_m'],
                     settle_rotation_rad=cfg['settle_rotation_rad']))
            if not checked['success']:raise ValueError(checked['reason'])
            if r['robot_state']!='AutoReady':raise ValueError('ROBOT_NOT_AUTOREADY')
            transforms.append(robot.pose_matrix(r['robot_pose_at_capture'])@np.asarray(calibration['T_flange_camera']))
        info=records[-1]['camera_info'];encoding=records[-1]['depth_encoding']
        if any(r['depth_encoding']!=encoding or r['camera_info']!=info for r in records):
            raise ValueError('PROFILE_CHANGED_DURING_CAPTURE')
        diagnostics={}
        candidate,debug,reason=locate(raw_frames,encoding,info,rgbs[-1],cfg,diagnostics=diagnostics)
        if candidate is None: quality['localization_diagnostics']=diagnostics
        quality['localization_reason']=reason
        frames=depth_metres(raw_frames,encoding);rays=rays_for(info)
        if candidate is None:
            # Disappearance is checked at the last measured target, not by
            # requiring a valid plane or complete depth over the entire image.
            if source_context is not None and reason == 'NO_MATCHING_RGBD_OBJECT':
                mask=np.asarray(source_context['object_mask'],bool)
                prior=np.asarray(source_context['background_gray'],np.uint8)
                if mask.shape != frames.shape[1:] or prior.shape != mask.shape:
                    return unknown('SOURCE_VIEW_SHAPE_CHANGED',quality=quality),debug
                background=~(cv2.dilate(mask.astype(np.uint8),np.ones((9,9),np.uint8))>0)
                delta=np.abs(cv2.cvtColor(rgbs[-1],cv2.COLOR_BGR2GRAY).astype(float)-prior)
                if not background.any() or np.mean(delta[background]>40)>.35:
                    return unknown('SOURCE_VIEW_CHANGED',quality=quality),debug
                old=np.asarray(source_context['point_base_m'])
                for f,tf in zip(frames,transforms):
                    q=tf[:3,:3].T@(old-tf[:3,3])
                    values=f[mask & np.isfinite(f)]
                    if len(values)<cfg['min_depth_points'] or np.median(values)<q[2]+cfg.get('source_clearance_depth_m',.015):
                        return unknown('SOURCE_NOT_CONFIRMED_CLEAR',quality=quality),debug
                return dict(presence='ABSENT',localizable=False,top_valid=False,
                            reason='PREVIOUS_TARGET_DEPTH_CLEARED',quality=quality),debug
            return unknown(reason,quality=quality),debug
        current=dict(records[-1],robot_pose_component_ranges=ranges)
        base=add_base_coordinates(candidate,current,records[-1]['depth_camera_info'],calibration,robot,
            dict(max_sync_sec=cfg['max_sync_s'],settle_translation_m=cfg['settle_translation_m'],
                 settle_rotation_rad=cfg['settle_rotation_rad']))
        if not base['success']:return unknown(base['reason'],quality=quality),debug
        result=dict(presence='PRESENT',localizable=False,top_valid=False,reason='TOP_NOT_ESTIMATED',
                    candidate_base_m=base['point_base_m'],candidate_method=candidate['point_kind'],
                    pixel=candidate['pixel'],object_bbox=candidate['object_bbox'],quality=quality,
                    rgbd_candidate=candidate)
        mask=debug['object_mask'].astype(bool)
        tops=[];details=[]
        for f,tf in zip(frames,transforms):
            accepted=mask & np.isfinite(f) & (np.abs(f-candidate['depth_m'])<=cfg.get('depth_layer_m',.012)*2)
            points=(rays*f[...,None])@tf[:3,:3].T+tf[:3,3]
            support=supported(points,accepted,cfg['top_local_span_m']);z=points[...,2][support]
            if len(z)<cfg['min_depth_points']:
                return dict(result,reason='OBJECT_TOP_DEPTH_UNSUPPORTED'),debug
            top=float(np.quantile(z,.95));band=int(np.count_nonzero(np.abs(z-top)<=cfg['top_local_span_m']))
            if band<cfg['min_depth_points']:return dict(result,reason='OBJECT_TOP_SUPPORT_INSUFFICIENT'),debug
            tops.append(top);details.append(dict(method='base_Z_95_percentile_of_camera_region_supported_3x3_points',
                z_top=top,support_points=len(z),top_band_points=band))
            debug['top_support']=support.astype(np.uint8)
        if np.ptp(tops)>cfg['top_temporal_span_m']:
            return dict(result,reason='OBJECT_TOP_TEMPORALLY_UNSTABLE'),debug
        result.update(localizable=True,top_valid=True,reason='OK',z_top=tops[-1],
            top_quality=dict(frames=details,temporal_span_m=float(np.ptp(tops))),
            source_context=dict(mode='camera',point_base_m=base['point_base_m'],
                background_gray=cv2.cvtColor(rgbs[-1],cv2.COLOR_BGR2GRAY).tolist(),object_mask=mask.tolist()))
        return result,debug
    except (ValueError,KeyError,TypeError,cv2.error,np.linalg.LinAlgError) as exc:
        return unknown(str(exc),quality=quality),debug


def tag(result,capture_id=None,captured_monotonic=None):
    return dict(result,capture_id=capture_id or str(uuid.uuid4()),
                captured_monotonic=time.monotonic() if captured_monotonic is None else captured_monotonic)
