"""All transforms T_a_b map coordinates in b into frame a; all lengths in m."""
import hashlib
import importlib.util
import math
import ast
import json
import os
import sys
from pathlib import Path

import numpy as np

BUNDLED_CORE = Path(__file__).with_name('estun_math.py')
PROVENANCE = Path(__file__).with_name('math_provenance.json')
CONVENTION = 'codroid_fixed_XYZ_RzRyRx'


def convention_matches(a,b):
    """The math identity matters; checkout directories and evidence fields may differ."""
    return (a.get('name')==b.get('name')==CONVENTION and
            isinstance(a.get('sha256'),str) and a['sha256']==b.get('sha256'))


def math_fingerprint(path,names):
    nodes=ast.parse(Path(path).read_text(encoding='utf-8')).body
    def stable(node):
        if isinstance(node,ast.AST):
            return [type(node).__name__,{k:stable(v) for k,v in ast.iter_fields(node) if k!='type_params'}]
        if isinstance(node,list): return [stable(v) for v in node]
        return node
    selected=[stable(n) for n in nodes
              if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    if len(selected)!=len(names): raise ValueError('Required robot math helpers are missing')
    return hashlib.sha256(json.dumps(selected,sort_keys=True,ensure_ascii=True).encode()).hexdigest()


def finite_vector(value, n):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError(f'Expected {n} finite components')
    return a


def transform(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Expected finite 4x4 transform')
    r = t[:3, :3]
    if (not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(r.T@r, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-7)):
        raise ValueError('Transform must have a proper SO(3) rotation and homogeneous last row')
    return t


def inverse(t):
    t = transform(t)
    out = np.eye(4); out[:3,:3] = t[:3,:3].T
    out[:3,3] = -out[:3,:3]@t[:3,3]
    return out


def rotation_angle(r):
    # atan2 retains precision for small angles.
    return math.atan2(np.linalg.norm([r[2,1]-r[1,2], r[0,2]-r[2,0],
                                     r[1,0]-r[0,1]])/2,
                      np.clip((np.trace(r)-1)/2, -1, 1))


class RobotConvention:
    """Load only the already verified pure math module, never runner/bridge."""
    def __init__(self, path=None):
        explicit=path or os.environ.get('ESTUN_CORE_PATH')
        sibling=Path(__file__).resolve().parents[2]/'estun/ros2_ws/src/estun_segmented/estun_segmented/core.py'
        self.path = Path(explicit).expanduser().resolve() if explicit else (sibling if sibling.is_file() else BUNDLED_CORE)
        if not self.path.is_file():
            raise ValueError(f'Existing robot core not found: {self.path}')
        self.loaded_sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        provenance=json.loads(PROVENANCE.read_text(encoding='utf-8'))
        signature=math_fingerprint(self.path,set(provenance['copied_symbols']))
        bundled_signature=math_fingerprint(BUNDLED_CORE,set(provenance['copied_symbols']))
        if bundled_signature!=provenance['math_ast_sha256']:
            raise ValueError('Bundled math changed: verify and update provenance before use')
        # Preserve the identity of the verified source for existing calibration
        # artifacts, including when only EOLs or checkout paths have changed.
        self.sha256 = provenance['original_sha256'] if signature==bundled_signature else self.loaded_sha256
        spec = importlib.util.spec_from_file_location('_existing_estun_math', self.path)
        mod = importlib.util.module_from_spec(spec); sys.modules[spec.name] = mod
        spec.loader.exec_module(mod)
        self.core = mod

    def evidence(self):
        return dict(name=CONVENTION, source=str(self.path), sha256=self.sha256, loaded_source_sha256=self.loaded_sha256,
                    official_reference='Codroid S series manual V1.0 section 9.2, PDF p125 / printed p107')

    def rotation(self, rpy):
        w,x,y,z = self.core.quaternion(finite_vector(rpy,3).tolist())
        return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                         [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                         [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

    def pose_matrix(self, pose):
        p = finite_vector(pose,6)
        t = np.eye(4); t[:3,:3] = self.rotation(p[3:]); t[:3,3] = p[:3]
        return transform(t)

    def components(self, position, rpy, reference_rpy=None):
        position = finite_vector(position,3); rpy = finite_vector(rpy,3)
        near = self.core.euler_near(self.core.quaternion(rpy.tolist()),
                    finite_vector(reference_rpy if reference_rpy is not None else rpy,3).tolist())
        if not np.allclose(self.rotation(near), self.rotation(rpy), atol=1e-8):
            raise ValueError('Existing Euler conversion failed rotation round-trip')
        return position.tolist()+list(near)

    def robotmove_preview(self, components):
        # Reuse verified joint_names, positions, speeds/accelerations; pure dict only.
        pose = finite_vector(components,6).tolist()
        return self.core.command(self.core.Segment('vision_preview', tuple(pose)), self.core.Settings())


def camera_to_base(point_camera, T_base_flange, T_flange_camera):
    p = finite_vector(point_camera,3)
    if p[2] <= 0:
        raise ValueError('Camera optical Z must be positive; zero depth is invalid')
    return (transform(T_base_flange)@transform(T_flange_camera)@np.r_[p,1])[:3]


def flange_to_gripper(offset=(0., 0., .15384)):
    t = np.eye(4); t[:3,3] = finite_vector(offset,3)
    return t


def compensate_gripper(point_base, R_base_flange_target, T_flange_gripper):
    """p_flange = p_object - R_target * t_flange_gripper (not current R)."""
    p = finite_vector(point_base,3)
    r = np.asarray(R_base_flange_target, dtype=float)
    check = np.eye(4); check[:3,:3] = r; transform(check)
    return p-r@transform(T_flange_gripper)[:3,3]


def select_grasp_point(point_base, point_confirmed=False, grasp_height_base_z_m=None,
                       use_top_surface_center=False):
    """Select the contact/gripping reference, never infer half the object height.

    An explicit height selects a NEW grasp point; it never repairs a flange Z
    violation and is recorded separately from the observed point.
    """
    measured=finite_vector(point_base,3)
    selected=measured.copy()
    if use_top_surface_center:
        if point_confirmed or grasp_height_base_z_m is not None:
            raise ValueError('Choose only one grasp point policy')
        mode='measured_top_surface_center'
    elif grasp_height_base_z_m is not None:
        if isinstance(grasp_height_base_z_m,bool) or not math.isfinite(grasp_height_base_z_m):
            raise ValueError('Grasp height must be finite base-frame Z in metres')
        selected[2]=grasp_height_base_z_m
        mode='operator_selected_base_z'
    elif point_confirmed is True:
        mode='operator_confirmed_3d_grasp_point'
    else:
        raise ValueError('GRASP_POINT_REQUIRED: select measured top surface center, confirm the supplied 3D grasp point, or explicitly supply base-frame grasp height')
    return selected,dict(mode=mode,observed_point_base_m=measured.tolist(),
                         selected_grasp_point_base_m=selected.tolist(),
                         height_difference_m=float(selected[2]-measured[2]),
                         is_flange_z_clamp=False)


def target_components(point_base, grasp_rpy, robot, offset=(0.,0.,.15384), reference_rpy=None,
                      min_flange_z=.213):
    if not math.isfinite(min_flange_z) or min_flange_z < .213:
        raise ValueError('Flange Z limit cannot be relaxed below 0.213 m')
    p = compensate_gripper(point_base, robot.rotation(grasp_rpy), flange_to_gripper(offset))
    original_pose = robot.components(p, grasp_rpy, reference_rpy)
    original_z_ok = bool(p[2] >= min_flange_z)
    z_adjustment = max(0., float(min_flange_z-p[2]))
    p[2] = max(float(p[2]), float(min_flange_z))
    pose = robot.components(p, grasp_rpy, reference_rpy)
    reconstructed = p+robot.rotation(pose[3:])@finite_vector(offset,3)
    error = float(np.linalg.norm(reconstructed-finite_vector(point_base,3)))
    ok = bool(p[2] >= min_flange_z)
    return dict(flange_pose=pose, original_flange_pose=original_pose,
                z_limit_ok=ok, original_z_limit_ok=original_z_ok,
                z_clamped=not original_z_ok, z_adjustment_m=z_adjustment,
                min_flange_z_m=min_flange_z,
                reason='' if original_z_ok else '法兰 Z 已提升至设定下限；夹爪参考点同步升高，不再与原目标点重合',
                requested_gripper_point_base_m=finite_vector(point_base,3).tolist(),
                gripper_point_error_base_m=(reconstructed-finite_vector(point_base,3)).tolist(),
                gripper_center_reconstructed_m=reconstructed.tolist(), compensation_error_m=error,
                robotmove_preview=robot.robotmove_preview(pose),
                executable=False, motion_sent=False)
