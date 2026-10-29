"""Connect a measured candidate to existing hand-eye math; no motion planning."""
import math
import numpy as np
from coordinate_transform.geometry import camera_to_base, convention_matches, finite_vector
from .grasp_candidate import aligned_profiles


def add_base_coordinates(candidate, context, depth_info, calibration, robot, config):
    if not candidate['success']: return candidate
    try:
        if not aligned_profiles(context['camera_info'],depth_info):
            raise ValueError('DEPTH_COLOR_PROFILE_MISMATCH')
        if (not aligned_profiles(calibration['camera_info'],depth_info) or
                context['camera_frame']!=depth_info['frame_id'] or
                calibration['camera_frame']!=depth_info['frame_id']):
            raise ValueError('HAND_EYE_CAMERA_PROFILE_MISMATCH')
        if calibration.get('status')!='validated_real' or calibration.get('heldout_consistency_verified') is not True:
            raise ValueError('REAL_VALIDATED_CALIBRATION_REQUIRED')
        if not convention_matches(calibration['convention'],robot.evidence()):
            raise ValueError('ROBOT_POSE_CONVENTION_MISMATCH')
        if context.get('robot_fault')!=0: raise ValueError('ROBOT_FEEDBACK_FAULT_OR_MISSING')
        stamps=[float(context[k]) for k in ('image_stamp','robot_stamp','capture_sync_error_sec')]
        if not all(math.isfinite(v) for v in stamps) or stamps[2]<0:
            raise ValueError('INVALID_CAPTURE_TIMESTAMPS')
        if max(abs(stamps[0]-stamps[1]),stamps[2])>config['max_sync_sec']:
            raise ValueError('CAPTURE_POSE_UNSYNCHRONIZED')
        ranges=finite_vector(context['robot_pose_component_ranges'],6)
        if (np.any(ranges<0) or np.linalg.norm(ranges[:3])>config['settle_translation_m'] or
                np.linalg.norm(ranges[3:])>config['settle_rotation_rad']):
            raise ValueError('ROBOT_POSE_UNSTABLE_DURING_BURST')
        pose=robot.pose_matrix(context['robot_pose_at_capture'])
        point=camera_to_base(candidate['point_camera_m'],pose,calibration['T_flange_camera'])
        return dict(candidate,point_base_m=point.tolist(),units='m',camera_frame=depth_info['frame_id'],
                    base_frame='robot_base',robot_pose_at_capture=context['robot_pose_at_capture'],
                    image_stamp=context['image_stamp'],robot_stamp=context['robot_stamp'],
                    robot_state_at_capture=context.get('robot_state'),replay=True)
    except (ValueError,KeyError,TypeError) as exc:
        return dict(success=False,reason=str(exc),point_kind=candidate.get('point_kind'),
                    segmentation=candidate.get('segmentation',{}),
                    executable=False,motion_sent=False)
