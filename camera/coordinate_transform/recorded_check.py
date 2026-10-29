"""Offline calculation artifacts; never opens camera, robot sockets or ROS nodes."""
import json
from pathlib import Path

from configs import load, save, DEFAULT_CONFIG, PROJECT_ROOT
from .geometry import RobotConvention, camera_to_base, target_components


def main():
    root=Path(__file__).resolve().parent
    config=load(DEFAULT_CONFIG); robot=RobotConvention(config['robot_core'])
    parent=PROJECT_ROOT/'results'
    calibration=load(parent/'handeye_real_20261023/calibration.json')
    out=parent/'downward_grasp_20261023'
    grasp=dict(grasp_rpy_rad=config['grasp_rpy_rad'],convention=robot.evidence(),
        operator_confirmed_for_calculation=True,gripper_offset_physically_verified=False,
        motion_verified=False,flange_to_gripper_m=config['flange_to_gripper_m'],
        jaw_mode='two_finger_side_closing',yaw_policy='fixed_at_user_specified_zero',
        source='User specified nominal rx=-180deg,ry=0deg,rz=0deg; +Z153.84mm provisional',
        grasp_point_selection='explicit gripping center or explicitly selected base-frame height required')
    save(out/'grasp_orientation.json',grasp)
    # Re-evaluate ONE recorded real depth point using its recorded flange pose.
    # A board-plane point is a boundary diagnostic, not a current solid object target.
    samples=[load(p) for p in sorted((parent/'handeye_real_20261023').glob('sample_*.json'))]
    sample=next(s for s in reversed(samples) if s['role']=='validation' and 'depth_board_center' in s)
    point=sample['depth_board_center']['point_camera_m']
    base=camera_to_base(point,robot.pose_matrix(sample['robot_pose']),calibration['T_flange_camera'])
    diagnostic=target_components(base,grasp['grasp_rpy_rad'],robot,config['flange_to_gripper_m'],
                                 grasp['grasp_rpy_rad'],config['min_flange_z_m'])
    diagnostic.update(data_kind='recorded_real_board_point_offline_height_diagnostic',
        source_sample_id=sample['id'],point_camera_m=point,point_base_m=base.tolist(),
        observed_robot_pose=sample['robot_pose'],observed_image_stamp=sample['image_stamp'],
        current_object_localization=False,actual_grasp_point_confirmed=False,
        gripper_offset_physically_verified=False,eligible_for_motion_review=False)
    save(out/'recorded_board_diagnostic.json',diagnostic)
    summary=dict(grasp_profile=str(out/'grasp_orientation.json'),
                 recorded_board_diagnostic=diagnostic,
                 minimum_gripping_height_base_z_m=config['min_flange_z_m']-config['flange_to_gripper_m'][2],
                 actual_solid_object_target_available=False,
                 note='No new real object center or selected gripping height provided; no motion/precision claim')
    save(out/'summary.json',summary)
    print(json.dumps(summary,indent=2,ensure_ascii=False))


if __name__=='__main__': main()
