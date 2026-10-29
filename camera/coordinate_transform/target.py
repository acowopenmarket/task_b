"""Metadata/height gates for read-only flange target calculation."""
from .geometry import camera_to_base, target_components, finite_vector, select_grasp_point, convention_matches


def plan_observation(observation,calibration,grasp,config,robot,allow_preview=False,
                     point_is_grasp_center=False,grasp_height_base_z_m=None,
                     use_top_surface_center=False):
    if observation.get('robot_fault')!=0: raise ValueError('Observation robot fault missing/nonzero')
    if observation.get('camera_frame')!=calibration['camera_frame']: raise ValueError('Camera optical frame mismatch')
    if observation.get('camera_info')!=calibration['camera_info']:
        raise ValueError('Camera image geometry differs; do not mix SDK medical profile and ROS profile')
    if (not convention_matches(observation.get('convention',{}),calibration['convention']) or
        not convention_matches(calibration['convention'],robot.evidence())):
        raise ValueError('Robot convention/source differs from calibration')
    if abs(observation['robot_stamp']-observation['image_stamp'])>config['max_sync_sec']:
        raise ValueError('Observation pose/image timestamps do not match')
    if observation.get('capture_sync_error_sec',float('inf'))>config['max_sync_sec']:
        raise ValueError('Observation was not captured synchronously')
    trusted=(calibration.get('status')=='validated_real' and calibration.get('heldout_consistency_verified') is True)
    approved=grasp.get('operator_confirmed_for_calculation') is True
    if not convention_matches(grasp.get('convention',{}),robot.evidence()): raise ValueError('Grasp convention mismatch')
    if not allow_preview and not (trusted and approved):
        raise ValueError('Need real held-out validated calibration and operator-confirmed grasp; numerical preview requires explicit --allow-unvalidated-preview')
    p=finite_vector(observation['point_camera_m'],3)
    base=camera_to_base(p,robot.pose_matrix(observation['robot_pose_at_capture']),calibration['T_flange_camera'])
    # Default to the operator-selected surface policy, without reinterpreting
    # an existing volume-center observation as a surface measurement.
    if (not point_is_grasp_center and grasp_height_base_z_m is None and
            config.get('grasp_point_selection')=='measured_top_surface_center'):
        use_top_surface_center=True
    if use_top_surface_center and observation.get('point_kind')!='top_surface_center':
        raise ValueError('TOP_SURFACE_CENTER_REQUIRED: input must be measured at the object top center; a volume center or arbitrary pixel cannot be relabelled')
    selected,selection=select_grasp_point(base,
        point_is_grasp_center or (not use_top_surface_center and observation.get('grasp_point_confirmed') is True),
        grasp_height_base_z_m,use_top_surface_center)
    result=target_components(selected,grasp['grasp_rpy_rad'],robot,config['flange_to_gripper_m'],
                             grasp['grasp_rpy_rad'],config['min_flange_z_m'])
    offset_verified=(grasp.get('gripper_offset_physically_verified',False) is True and
                     config.get('gripper_offset_physically_verified',False) is True)
    result.update(point_camera_m=p.tolist(),point_base_m=base.tolist(),
        calibration_status=calibration['status'],observation_source=observation.get('data_kind'),
        calibration_heldout_verified=trusted,grasp_operator_confirmed=approved,
        eligible_for_motion_review=bool(trusted and approved and result['z_limit_ok'] and offset_verified),
        grasp_point_selection=selection,
        selected_grasp_point_base_m=selected.tolist(),
        gripper_offset_physically_verified=offset_verified,
        calculation_uses_provisional_gripper_offset=not offset_verified,
        grasp_mode='fixed_orientation_two_finger_side_closing',
        absolute_localization_accuracy_verified=False,
        note='Read-only snapshot calculation at image capture pose. No reachability/collision/motion validation.')
    if not trusted or not approved:
        result['robotmove_preview']=None
    return result
