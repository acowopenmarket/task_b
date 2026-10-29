import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from coordinate_transform.geometry import transform, inverse, rotation_angle


def board_points(board):
    cols, rows = board['inner_columns'], board['inner_rows']
    size = board['square_size_m']
    if cols != 11 or rows != 8 or not math.isfinite(size) or size <= 0:
        raise ValueError('Expected supplied 11x8 inner corners with positive measured square size')
    p = np.zeros((cols*rows,3), np.float64)
    p[:,:2] = np.mgrid[:cols,:rows].T.reshape(-1,2)*size
    return p


def detect_board(bgr, camera_info, board, reverse=False):
    k = np.asarray(camera_info['k'],float).reshape(3,3)
    d = np.asarray(camera_info['d'],float)
    if camera_info['distortion_model'] not in ('plumb_bob','rational_polynomial'):
        raise ValueError('Unsupported camera distortion model')
    if k[0,0] <= 0 or k[1,1] <= 0 or bgr.shape[:2] != (camera_info['height'],camera_info['width']):
        raise ValueError('Image/CameraInfo dimensions or calibration invalid')
    gray = cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray,(board['inner_columns'],board['inner_rows']),
                                  flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        raise ValueError('CHESSBOARD_NOT_FOUND: need all 11x8 inner corners')
    if reverse: corners = corners[::-1].copy()
    obj = board_points(board)
    retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj,corners,k,d,flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if retval:
        for rv,tv in zip(rvecs,tvecs):
            r,_ = cv2.Rodrigues(rv)
            if np.min((r@obj.T+tv.reshape(3,1))[2]) <= 0: continue
            projected,_ = cv2.projectPoints(obj,rv,tv,k,d)
            rms = float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=2))))
            candidates.append((rms,r,tv.reshape(3)))
    if not candidates: raise ValueError('PNP_FAILED_OR_BOARD_BEHIND_CAMERA')
    candidates.sort(key=lambda x:x[0])
    if len(candidates)>1:
        separation = math.degrees(rotation_angle(candidates[0][1].T@candidates[1][1]))
        if candidates[1][0]-candidates[0][0]<.05 and separation>5:
            raise ValueError('PLANAR_PNP_AMBIGUOUS: tilt board view slightly and recapture')
    _,r,t = candidates[0]
    rv,_=cv2.Rodrigues(r)
    rv,tv=cv2.solvePnPRefineLM(obj,corners,k,d,rv,t.reshape(3,1))
    r,_=cv2.Rodrigues(rv); t=tv.reshape(3)
    projected,_=cv2.projectPoints(obj,rv,tv,k,d)
    rms=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=2))))
    if np.min((r@obj.T+t[:,None])[2])<=0: raise ValueError('REFINED_BOARD_BEHIND_CAMERA')
    result = np.eye(4); result[:3,:3]=r; result[:3,3]=t
    return transform(result), corners, rms


def pose_coverage(samples, robot):
    transforms = [robot.pose_matrix(s['robot_pose']) for s in samples]
    base = transforms[0]
    vectors = np.array([cv2.Rodrigues(base[:3,:3].T@t[:3,:3])[0].reshape(3) for t in transforms[1:]])
    singular = np.linalg.svd(vectors,compute_uv=False)
    span = max(rotation_angle(base[:3,:3].T@t[:3,:3]) for t in transforms)
    if len(singular)<2 or singular[1]<.15 or span<math.radians(15):
        raise ValueError('DEGENERATE_POSES: need substantial rotations about nonparallel axes')
    return dict(rotation_vector_singular_values=singular.tolist(), max_rotation_deg=math.degrees(span))


def metrics(samples, robot, x, reference):
    locations = [robot.pose_matrix(s['robot_pose'])@x@transform(s['T_camera_board']) for s in samples]
    distances = [float(np.linalg.norm(t[:3,3]-reference[:3,3])) for t in locations]
    angles = [math.degrees(rotation_angle(reference[:3,:3].T@t[:3,:3])) for t in locations]
    # Validate all board corners in base coordinates, not just one origin.
    obj = board_points(samples[0]['board'])
    refpoints = (reference[:3,:3]@obj.T+reference[:3,3,None]).T
    errors = [np.linalg.norm((t[:3,:3]@obj.T+t[:3,3,None]).T-refpoints,axis=1) for t in locations]
    return dict(count=len(samples), origin_rms_m=float(np.sqrt(np.mean(np.square(distances)))),
                origin_max_m=max(distances), rotation_rms_deg=float(np.sqrt(np.mean(np.square(angles)))),
                rotation_max_deg=max(angles), all_corners_rms_m=float(np.sqrt(np.mean(np.square(errors)))),
                all_corners_max_m=float(np.max(errors)), origin_errors_m=distances)


def validate_samples(samples, robot):
    if not samples: raise ValueError('No samples')
    first=samples[0]
    ids=set()
    for s in samples:
        if s['id'] in ids: raise ValueError('Duplicate sample ID')
        ids.add(s['id'])
        if s['role'] not in ('train','validation'): raise ValueError('Unknown sample role')
        if s['data_kind'] not in ('real_checkerboard','synthetic'): raise ValueError('Unknown provenance')
        for key in ('board','camera_info','camera_frame','convention','data_kind','session_id'):
            if s[key] != first[key]: raise ValueError('Mixed data/configuration: '+key)
        if s['convention']['sha256']!=robot.sha256: raise ValueError('Robot math source changed')
        if not s.get('origin_confirmed'): raise ValueError('Checkerboard origin not confirmed')
        if s.get('robot_fault')!=0: raise ValueError('Sample contains robot fault')
        transform(s['T_camera_board']); robot.pose_matrix(s['robot_pose'])
        if not math.isfinite(s['reprojection_rms_px']): raise ValueError('Invalid reprojection error')


def solve(samples, config, robot):
    validate_samples(samples,robot)
    train=[s for s in samples if s['role']=='train']; val=[s for s in samples if s['role']=='validation']
    if len(train)<config['min_training_poses'] or len(val)<config['min_validation_poses']:
        raise ValueError(f'Need >= {config["min_training_poses"]} training and >= {config["min_validation_poses"]} held-out poses')
    if any(s['reprojection_rms_px']>config['max_reprojection_px'] for s in samples):
        raise ValueError('Sample reprojection error exceeds configured threshold')
    coverage=pose_coverage(train,robot)
    bt=[robot.pose_matrix(s['robot_pose']) for s in train]
    ct=[transform(s['T_camera_board']) for s in train]
    candidates=[]
    for name,method in [('PARK',cv2.CALIB_HAND_EYE_PARK),('TSAI',cv2.CALIB_HAND_EYE_TSAI),
                        ('HORAUD',cv2.CALIB_HAND_EYE_HORAUD)]:
        try:
            r,t=cv2.calibrateHandEye([a[:3,:3] for a in bt],[a[:3,3] for a in bt],
                                     [a[:3,:3] for a in ct],[a[:3,3] for a in ct],method=method)
            x=np.eye(4); x[:3,:3]=r; x[:3,3]=t.reshape(3); transform(x)
            references=[a@x@b for a,b in zip(bt,ct)]
            # Medoid chosen ONLY from training data. Validation never tunes method or fit.
            ref=min(references,key=lambda a:sum(np.linalg.norm(a[:3,3]-b[:3,3])+
                               .1*rotation_angle(a[:3,:3].T@b[:3,:3]) for b in references))
            candidates.append(dict(method=name,T_flange_camera=x.tolist(),T_base_board=ref.tolist(),
                training=metrics(train,robot,x,ref),validation=metrics(val,robot,x,ref)))
        except (cv2.error,ValueError): continue
    if not candidates: raise ValueError('All hand-eye solvers failed')
    best=min(candidates,key=lambda c:c['training']['all_corners_rms_m'])
    validation=best['validation']
    passed=(validation['all_corners_rms_m']<=config['max_validation_rms_m'] and
            validation['all_corners_max_m']<=config['max_validation_error_m'] and
            validation['rotation_rms_deg']<=config['max_validation_rotation_deg'])
    real=samples[0]['data_kind']=='real_checkerboard'
    return dict(schema_version=1, status=('validated_real' if passed and real else
                'simulation_only' if not real else 'failed_validation'),
                heldout_consistency_verified=bool(passed and real), real_accuracy_verified=False,
                validation_scope='held-out fixed-board consistency only; absolute object localization and motion unverified',
                convention=robot.evidence(),camera_frame=samples[0]['camera_frame'],
                camera_info=samples[0]['camera_info'],board=samples[0]['board'],
                session_id=samples[0]['session_id'],data_kind=samples[0]['data_kind'],
                sample_ids=[s['id'] for s in samples],coverage=coverage,
                selected_method=best['method'],T_flange_camera=best['T_flange_camera'],
                T_base_board=best['T_base_board'],training=best['training'],validation=validation,
                methods=candidates, thresholds={k:config[k] for k in config if k.startswith('max_validation')})


def read_samples(directory):
    return [json.loads(p.read_text()) for p in sorted(Path(directory).glob('sample_*.json'))]
