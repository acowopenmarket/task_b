import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from coordinate_transform.geometry import RobotConvention, camera_to_base
from coordinate_transform.target import plan_observation
from vision_center.adapters import medical_center
from .calibration import read_samples, solve, transform

from configs import DEFAULT_CONFIG, load, save


def fixed_depth_validation(samples,result,robot):
    val=[s for s in samples if s['role']=='validation' and 'depth_board_center' in s]
    if not val: return dict(count=0,verified=False,reason='No held-out aligned depth observations')
    points=[]; expected=[]
    ref=transform(result['T_base_board'])
    for s in val:
        item=s['depth_board_center']
        points.append(camera_to_base(item['point_camera_m'],robot.pose_matrix(s['robot_pose']),result['T_flange_camera']))
        expected.append((ref@np.r_[item['expected_point_board_m'],1])[:3])
    points=np.array(points); errors=np.linalg.norm(points-np.array(expected),axis=1)
    center=np.median(points,axis=0)
    return dict(count=len(points),median_point_base_m=center.tolist(),
                spread_rms_m=float(np.sqrt(np.mean(np.linalg.norm(points-center,axis=1)**2))),
                board_reference_rms_m=float(np.sqrt(np.mean(errors**2))),board_reference_max_m=float(errors.max()),
                verified=bool(result['data_kind']=='real_checkerboard' and len(points)>=5 and errors.max()<=.005),
                note='Fixed board center depth vs held-out board reference; not external metrology')


def main():
    p=argparse.ArgumentParser(description='Read-only S3-60 Pro eye-in-hand tools: no motion command exists')
    p.add_argument('--config',default=str(DEFAULT_CONFIG))
    sub=p.add_subparsers(dest='command',required=True)
    cap=sub.add_parser('capture'); cap.add_argument('--dataset',required=True); cap.add_argument('--duration',type=float,default=0)
    sol=sub.add_parser('solve'); sol.add_argument('--dataset',required=True); sol.add_argument('--output',required=True)
    tar=sub.add_parser('target'); tar.add_argument('--observation',required=True); tar.add_argument('--calibration',required=True)
    tar.add_argument('--grasp',required=True); tar.add_argument('--output',required=True)
    tar.add_argument('--allow-unvalidated-preview',action='store_true')
    selection=tar.add_mutually_exclusive_group()
    selection.add_argument('--use-top-surface-center',action='store_true',help='Use measured object top center directly, without subtracting half the object height; requires observation point_kind=top_surface_center')
    selection.add_argument('--point-is-grasp-center',action='store_true',help='Explicitly confirm the input is the selected gripping center, not just visible surface center')
    selection.add_argument('--grasp-height-base-z',type=float,help='Explicitly selected gripping height in base frame, metres; not flange Z')
    med=sub.add_parser('medical-observation'); med.add_argument('--item',required=True)
    med.add_argument('--capture-context',required=True); med.add_argument('--output',required=True)
    args=p.parse_args(); config=load(args.config); robot=RobotConvention(config['robot_core'])
    try:
        if args.command=='capture':
            from .capture import run
            run(config,robot,args.dataset,args.duration)
        elif args.command=='solve':
            samples=read_samples(args.dataset); result=solve(samples,config,robot)
            result['depth_fixed_point_validation']=fixed_depth_validation(samples,result,robot)
            result['dataset_sha256']=hashlib.sha256(json.dumps(samples,sort_keys=True).encode()).hexdigest()
            save(args.output,result)
            print(json.dumps({k:result[k] for k in ('status','selected_method','training','validation','depth_fixed_point_validation')},indent=2))
        elif args.command=='target':
            result=plan_observation(load(args.observation),load(args.calibration),load(args.grasp),config,robot,
                args.allow_unvalidated_preview,args.point_is_grasp_center,args.grasp_height_base_z,
                args.use_top_surface_center)
            save(args.output,result)
            print('Target flange pose (READ ONLY):')
            for name,value,unit in zip(('x','y','z','rx','ry','rz'),result['flange_pose'],('m','m','m','rad','rad','rad')):
                print(f'{name:2} = {value:.9f} {unit}')
            print(json.dumps(result,indent=2,ensure_ascii=False))
        elif args.command=='medical-observation':
            item=load(args.item); context=load(args.capture_context)
            if (context.get('source_image_matches_recognition') is not True or
                context.get('medical_depth_geometry_verified') is not True):
                raise ValueError('Medical SDK geometry/depth units and matching source frame not verified; old API has no capture timestamp')
            context.update(point_camera_m=medical_center(item).tolist(),data_kind='medical_recognition_center_mm_to_m')
            save(args.output,context)
    except (ValueError,KeyError,OSError) as exc: p.exit(2,'REFUSED: '+str(exc)+'\n')


if __name__=='__main__': main()
